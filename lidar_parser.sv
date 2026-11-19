`timescale 1ns/1ps

// Module: lidar_parser
// Description: Turns the raw byte stream from uart_rx into decoded RPLIDAR points.
//
// Expected stream after the host sends the SCAN command (A5 20):
//
//   1) A 7-byte response descriptor, sent ONCE:   A5 5A 05 00 00 40 81
//   2) Then an endless stream of 5-byte "nodes", one per laser measurement:
//
//        byte0: [q5 q4 q3 q2 q1 q0 | !S | S ]   quality (6b), S = new-scan flag, !S = its inverse
//        byte1: [a6 a5 a4 a3 a2 a1 a0 | C=1 ]   angle bits 6..0, check bit (always 1)
//        byte2: [a14 a13 ...        a8 a7   ]   angle bits 14..7
//        byte3: [d7  d6  ...        d1  d0  ]   distance LSB
//        byte4: [d15 d14 ...        d9  d8  ]   distance MSB
//
//      angle_q6 = {byte2, byte1[7:1]}  -> degrees = angle_q6 / 64
//      dist_q2  = {byte4, byte3}       -> millimeters = dist_q2 / 4
//
// This module is a pure decoder: every node is passed through, including ones
// with dist_q2 == 0 ("no return"). Filtering those out is the next stage's job,
// because the new-scan flag can ride on a distance-0 node and must not be lost.
module lidar_parser (
    input  logic        clk,
    input  logic        rst,          // Synchronous, active-high reset
    input  logic        byte_valid,   // 1-clock strobe from uart_rx: byte_in holds a new byte
    input  logic [7:0]  byte_in,      // Received byte from uart_rx
    output logic        point_valid,  // 1-clock strobe: the outputs below hold one decoded point
    output logic [15:0] angle_q6,     // Angle in 1/64 degree units (bit 15 is always 0)
    output logic [15:0] dist_q2,      // Distance in 1/4 mm units (0 = no return)
    output logic [5:0]  quality,      // Reflectivity / signal quality
    output logic        new_scan,     // Valid with point_valid: this point starts a new 360-degree scan
    output logic        desc_ok       // 1-clock strobe: the 7-byte descriptor was recognized
);

    // ------------------------------------------------------------------
    // The descriptor we expect, one byte at a time (index 0..6).
    // A function with a case is used instead of an array constant so that
    // every simulator and synthesis tool accepts it.
    // ------------------------------------------------------------------
    function automatic logic [7:0] desc_byte(input logic [2:0] i);
        case (i)
            3'd0:    desc_byte = 8'hA5;
            3'd1:    desc_byte = 8'h5A;
            3'd2:    desc_byte = 8'h05;
            3'd3:    desc_byte = 8'h00;
            3'd4:    desc_byte = 8'h00;
            3'd5:    desc_byte = 8'h40;
            default: desc_byte = 8'h81;   // index 6: data type 0x81 = "scan data follows"
        endcase
    endfunction

    // ------------------------------------------------------------------
    // State machine:
    //   S_DESC : looking for the descriptor (A5 5A 05 00 00 40 81).
    //   S_NODE : assembling 5-byte nodes forever.
    // After reset we sit in S_DESC. Pulse rst again if you restart the scan
    // (send a new A5 20), because a fresh descriptor will arrive.
    // ------------------------------------------------------------------
    typedef enum logic {S_DESC, S_NODE} state_t;
    state_t state;

    logic [2:0] idx;       // S_DESC: how many descriptor bytes matched so far (0..6)
                           // S_NODE: which byte of the node we expect next (0..4)
    logic [7:0] b0, b1, b2, b3;  // Holding registers for the first four node bytes.
                                 // byte4 is never stored: it is used straight from byte_in.

    // Sync checks, written as wires so the intent is readable below.
    // byte0 is plausible only if S and !S differ (bit0 != bit1).
    wire byte0_ok = byte_in[0] ^ byte_in[1];
    // byte1 is plausible only if its check bit (bit 0) is 1.
    wire byte1_ok = byte_in[0];

    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= S_DESC;
            idx         <= '0;
            b0          <= '0;
            b1          <= '0;
            b2          <= '0;
            b3          <= '0;
            point_valid <= 1'b0;
            angle_q6    <= '0;
            dist_q2     <= '0;
            quality     <= '0;
            new_scan    <= 1'b0;
            desc_ok     <= 1'b0;
        end else begin
            // Default: strobes are low unless explicitly pulsed this cycle.
            point_valid <= 1'b0;
            desc_ok     <= 1'b0;

            // Only do work when uart_rx hands us a fresh byte.
            if (byte_valid) begin
                case (state)

                    // ----------------------------------------------------------
                    // S_DESC: match the 7-byte descriptor in order.
                    // On a mismatch we restart the match. If the offending byte is
                    // itself 0xA5, it may be the START of the descriptor, so we
                    // count it as a match of byte 0 instead of throwing it away.
                    // ----------------------------------------------------------
                    S_DESC: begin
                        if (byte_in == desc_byte(idx)) begin
                            if (idx == 3'd6) begin
                                desc_ok <= 1'b1;   // Whole descriptor seen
                                state   <= S_NODE; // From now on, bytes are nodes
                                idx     <= '0;     // Next byte is node byte 0
                            end else begin
                                idx <= idx + 1'b1;
                            end
                        end else begin
                            idx <= (byte_in == 8'hA5) ? 3'd1 : 3'd0;
                        end
                    end

                    // ----------------------------------------------------------
                    // S_NODE: collect 5 bytes per measurement, checking sync bits.
                    // ----------------------------------------------------------
                    S_NODE: begin
                        case (idx)
                            // Byte 0: only accept it if S != !S. If not, we are out
                            // of sync: drop the byte and stay at index 0, so we keep
                            // sliding one byte at a time until a plausible byte0 shows up.
                            3'd0: begin
                                if (byte0_ok) begin
                                    b0  <= byte_in;
                                    idx <= 3'd1;
                                end
                            end

                            // Byte 1: its check bit must be 1. If it fails, the byte0
                            // we kept was a false alarm. The current byte might itself
                            // be a real byte0, so re-test it as such before giving up.
                            3'd1: begin
                                if (byte1_ok) begin
                                    b1  <= byte_in;
                                    idx <= 3'd2;
                                end else if (byte0_ok) begin
                                    b0  <= byte_in;   // Treat this byte as a new byte0
                                    idx <= 3'd1;
                                end else begin
                                    idx <= 3'd0;
                                end
                            end

                            // Bytes 2 and 3 carry no sync bits, so just store them.
                            3'd2: begin
                                b2  <= byte_in;
                                idx <= 3'd3;
                            end

                            3'd3: begin
                                b3  <= byte_in;
                                idx <= 3'd4;
                            end

                            // Byte 4: the node is complete. byte4 is still on byte_in
                            // this cycle (not stored anywhere), so we use it directly
                            // while building the outputs, then pulse point_valid.
                            3'd4: begin
                                quality     <= b0[7:2];              // Top 6 bits of byte0
                                new_scan    <= b0[0];                // S flag
                                angle_q6    <= {1'b0, b2, b1[7:1]};  // 15-bit angle, zero-extended to 16
                                dist_q2     <= {byte_in, b3};        // MSB (byte4) : LSB (byte3)
                                point_valid <= 1'b1;
                                idx         <= 3'd0;                 // Ready for the next node
                            end

                            default: idx <= 3'd0; // Unreachable (idx is only 0..4); safe recovery
                        endcase
                    end
                endcase
            end
        end
    end

endmodule // lidar_parser