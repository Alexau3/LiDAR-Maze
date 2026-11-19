`timescale 1ns/1ps

// Module: lidar_ctrl
// Description: Startup / supervision FSM for the RPLIDAR C1.
//
//   power-up -> wait for lidar to boot -> send STOP (A5 25) -> let it settle
//   -> reset the parser -> send SCAN (A5 20) -> wait for the descriptor
//   -> RUN (watch that points keep flowing).
//
// If the descriptor never shows up, or points stop arriving for too long,
// it quietly starts the whole sequence over and bumps a 'restarts' counter.
//
// Why STOP first? The lidar keeps scanning if only the FPGA was reconfigured.
// STOP puts it in a known idle state before we ask for a fresh scan.
//
// All timeouts are in milliseconds and converted to clock cycles below, so they
// scale with CLK_FREQ. The defaults are conservative guesses: tune on hardware
// (the motor needs time to spin up, so points may not flow instantly).
module lidar_ctrl #(
    parameter int CLK_FREQ        = 100_000_000, // System clock in Hz
    parameter int STARTUP_MS      = 500,         // Wait after reset before talking to the lidar
    parameter int STOP_SETTLE_MS  = 10,          // Pause after STOP before sending SCAN
    parameter int DESC_TIMEOUT_MS = 2000,        // Max wait for the response descriptor
    parameter int WATCHDOG_MS     = 1000         // Max silence (no points) while running
)(
    input  logic       clk,
    input  logic       rst,          // Synchronous, active-high reset

    // --- to the 460800-baud uart_tx that drives the lidar's RX pin ---
    output logic       tx_start,     // 1-clock strobe: send tx_data
    output logic [7:0] tx_data,
    input  logic       tx_busy,      // uart_tx busy flag

    // --- from lidar_parser ---
    input  logic       desc_ok,      // Descriptor seen
    input  logic       point_valid,  // A decoded point arrived (used as a heartbeat)

    // --- to lidar_parser ---
    output logic       parser_rst,   // 1-clock pulse. OR it into the parser's rst: .rst(rst | parser_rst)

    // --- status (LEDs / debug) ---
    output logic       scanning,     // HIGH while in RUN
    output logic [7:0] restarts      // Number of automatic restarts (saturates at 255)
);

    // ------------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------------
    localparam logic [7:0] CMD_HDR  = 8'hA5;  // Every request starts with A5
    localparam logic [7:0] CMD_STOP = 8'h25;  // Stop scanning (no response)
    localparam logic [7:0] CMD_SCAN = 8'h20;  // Start scanning (response: descriptor + nodes)

    localparam int CYC_PER_MS     = CLK_FREQ / 1000;
    localparam int STARTUP_CYC    = CYC_PER_MS * STARTUP_MS;
    localparam int SETTLE_CYC     = CYC_PER_MS * STOP_SETTLE_MS;
    localparam int DESC_TMO_CYC   = CYC_PER_MS * DESC_TIMEOUT_MS;
    localparam int WATCHDOG_CYC   = CYC_PER_MS * WATCHDOG_MS;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_BOOT,       // Wait for the lidar to power up
        S_SEND,       // Wait until uart_tx is free, then hand it one byte
        S_SEND_WAIT,  // Wait for uart_tx to START sending that byte (busy rises)
        S_SEND_DRAIN, // Wait for uart_tx to FINISH that byte (busy falls)
        S_SETTLE,     // Give the lidar time to act on STOP
        S_WAIT_DESC,  // SCAN sent; wait for the parser to see the descriptor
        S_RUN         // Streaming; watchdog on point_valid
    } state_t;
    state_t state;

    logic        phase;     // 0 = sending STOP, 1 = sending SCAN
    logic        byte_sel;  // 0 = sending the A5 header byte, 1 = sending the command byte
    logic [31:0] cnt;       // General-purpose timer (reused by BOOT / SETTLE / WAIT_DESC / RUN)

    // Which command byte goes out for the current phase
    wire [7:0] cmd = phase ? CMD_SCAN : CMD_STOP;

    assign scanning = (state == S_RUN);

    // Start the whole sequence over (used by both timeouts). Written as a task-like
    // block of assignments inside the always_ff below; restarts saturates at 255.

    always_ff @(posedge clk) begin
        if (rst) begin
            state      <= S_BOOT;
            phase      <= 1'b0;
            byte_sel   <= 1'b0;
            cnt        <= '0;
            tx_start   <= 1'b0;
            tx_data    <= '0;
            parser_rst <= 1'b0;
            restarts   <= '0;
        end else begin
            // Defaults: strobes are low unless set this cycle
            tx_start   <= 1'b0;
            parser_rst <= 1'b0;

            case (state)

                // Wait for the lidar to come up, then begin with STOP.
                S_BOOT: begin
                    if (cnt == 32'(STARTUP_CYC - 1)) begin
                        cnt      <= '0;
                        phase    <= 1'b0;   // STOP first
                        byte_sel <= 1'b0;   // header byte first
                        state    <= S_SEND;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // Hand one byte (A5, then the command) to uart_tx.
                // uart_tx ignores 'start' while busy, so only fire when it is free.
                S_SEND: begin
                    if (!tx_busy) begin
                        tx_data  <= byte_sel ? cmd : CMD_HDR;
                        tx_start <= 1'b1;            // Visible to uart_tx next cycle
                        state    <= S_SEND_WAIT;
                    end
                end

                // uart_tx only raises busy one cycle AFTER it sees 'start', so first
                // wait for busy to go HIGH. (Checking !busy here would see the stale
                // "free" value and fire a second byte on top of the first.)
                S_SEND_WAIT: begin
                    if (tx_busy) state <= S_SEND_DRAIN;
                end

                // ...then wait for the byte to finish (busy LOW) and decide what is next.
                S_SEND_DRAIN: begin
                    if (!tx_busy) begin
                        if (!byte_sel) begin
                            byte_sel <= 1'b1;        // Header done; now the command byte
                            state    <= S_SEND;
                        end else begin
                            byte_sel <= 1'b0;        // Both bytes done
                            cnt      <= '0;
                            state    <= phase ? S_WAIT_DESC : S_SETTLE;
                        end
                    end
                end

                // After STOP, wait a bit, clear the parser, then send SCAN.
                S_SETTLE: begin
                    if (cnt == 32'(SETTLE_CYC - 1)) begin
                        cnt        <= '0;
                        parser_rst <= 1'b1;   // Parser must be hunting for a fresh descriptor
                        phase      <= 1'b1;   // SCAN next
                        state      <= S_SEND;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // SCAN has been sent. The parser pulses desc_ok when it sees the descriptor.
                S_WAIT_DESC: begin
                    if (desc_ok) begin
                        cnt   <= '0;
                        state <= S_RUN;
                    end else if (cnt == 32'(DESC_TMO_CYC - 1)) begin
                        // No descriptor: start over from STOP
                        cnt      <= '0;
                        phase    <= 1'b0;
                        byte_sel <= 1'b0;
                        if (restarts != 8'hFF) restarts <= restarts + 1'b1;
                        state    <= S_SEND;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // Running. Every decoded point is a heartbeat that clears the timer.
                S_RUN: begin
                    if (point_valid) begin
                        cnt <= '0;
                    end else if (cnt == 32'(WATCHDOG_CYC - 1)) begin
                        // Points stopped: start over from STOP
                        cnt      <= '0;
                        phase    <= 1'b0;
                        byte_sel <= 1'b0;
                        if (restarts != 8'hFF) restarts <= restarts + 1'b1;
                        state    <= S_SEND;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: state <= S_BOOT;
            endcase
        end
    end

endmodule // lidar_ctrl