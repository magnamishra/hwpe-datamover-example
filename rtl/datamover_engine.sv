/*
 * Copyright (C) 2020 ETH Zurich and University of Bologna
 *
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

/*
 * Authors:  Francesco Conti <f.conti@unibo.it>
             Magna Mishra < Add dummy processing capabilities for Wakelet >
 */

/*
  - Changes 
  - Update state machine to update frame buffer only in FILL state 
  - Mske pixel_wakeup_o level triggered
*/

import hwpe_stream_package::*;
import hci_package::*;

module datamover_engine #(
  parameter int unsigned FIFO_DEPTH = 4,
  parameter int unsigned BW_ALIGNED = 32
) (
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,
  input logic [31:0]             pixel_diff_threshold_i,
  // output
  output logic                   pixel_wakeup_o,
  // input data stream + handshake
  hwpe_stream_intf_stream.sink   data_in,
  // output data stream + handshake
  hwpe_stream_intf_stream.source data_out
);

  localparam int unsigned PIXELS_PER_WORD = BW_ALIGNED / 8;
  localparam int unsigned WORDS_PER_FRAME = 4096 / PIXELS_PER_WORD;
  localparam int unsigned WORD_CNT_WIDTH  = $clog2(WORDS_PER_FRAME);
  localparam int unsigned DIFF_CNT_WIDTH  = $clog2(WORDS_PER_FRAME * PIXELS_PER_WORD + 1);

  // Frame buffer holds previous frame for comparison
  logic [BW_ALIGNED-1:0] frame_buf [0:WORDS_PER_FRAME-1];

  // Internal signals
  logic [WORD_CNT_WIDTH-1:0] word_cnt_d, word_cnt_q;
  logic [DIFF_CNT_WIDTH-1:0] diff_cnt_d, diff_cnt_q;
  logic                       last_word;
  logic                       word_valid;
  logic [DIFF_CNT_WIDTH-1:0] word_diff_count;

  assign word_valid = data_in.valid & data_in.ready;
  assign last_word  = (word_cnt_q == WORD_CNT_WIDTH'(WORDS_PER_FRAME - 1));

  typedef enum logic [1:0] {
    FILL    = 2'd0,
    COMPARE = 2'd1
  } state_t;

  state_t state_d, state_q;

  // Pixel difference computation uses frame_buf combinationally (old value)
  // frame_buf write happens on clock edge so old value is always used here
  always_comb begin : pixel_compare
    word_diff_count = '0;
    for (int i = 0; i < PIXELS_PER_WORD; i++) begin
      if (data_in.data[i*8 +: 8] != frame_buf[word_cnt_q][i*8 +: 8]) begin
        word_diff_count = word_diff_count + 1;
      end
    end
  end

  // Sequential block
  always_ff @(posedge clk_i or negedge rst_ni) begin : fsm_seq
    if (!rst_ni) begin
      state_q    <= FILL;
      word_cnt_q <= '0;
      diff_cnt_q <= '0;
    end else begin
      state_q    <= state_d;
      word_cnt_q <= word_cnt_d;
      diff_cnt_q <= diff_cnt_d;
    end
  end

  // Next state logic
  always_comb begin : next_state
    state_d    = state_q;
    word_cnt_d = word_cnt_q;
    diff_cnt_d = diff_cnt_q;

    case (state_q)
      FILL: begin
        if (word_valid) begin
          word_cnt_d = word_cnt_q + 1;
          if (last_word) begin
            word_cnt_d = '0;
            state_d    = COMPARE;
          end
        end
      end

      COMPARE: begin
        if (word_valid) begin
          diff_cnt_d = diff_cnt_q + word_diff_count;
          word_cnt_d = word_cnt_q + 1;
          if (last_word) begin
            word_cnt_d = '0;
            diff_cnt_d = '0;
          end
        end
      end

      default: state_d = FILL;
    endcase
  end

  // Frame buffer update:
  // During FILL: store incoming frame as reference
  // During COMPARE: update frame_buf with current frame AFTER comparison
  //   - pixel_compare block reads frame_buf combinationally (old value)
  //   - this write stores new value for next comparison
  always_ff @(posedge clk_i) begin : frame_buf_write
    if (word_valid) begin
      frame_buf[word_cnt_q] <= data_in.data;
    end
  end

  // Wakeup generation level triggered, held until clear
  always_ff @(posedge clk_i or negedge rst_ni) begin : wakeup_gen
    if (!rst_ni) begin
      pixel_wakeup_o <= 1'b0;
    end else if (clear_i) begin
      pixel_wakeup_o <= 1'b0;
    end else begin
      if (state_q == COMPARE && word_valid && last_word) begin
        if ((diff_cnt_q + word_diff_count) > pixel_diff_threshold_i) begin
          pixel_wakeup_o <= 1'b1;
        end
      end
    end
  end

  // Output FIFO
  hwpe_stream_fifo #(
    .DATA_WIDTH ( BW_ALIGNED ),
    .FIFO_DEPTH ( FIFO_DEPTH )
  ) i_fifo (
    .clk_i   ( clk_i   ),
    .rst_ni  ( rst_ni  ),
    .clear_i ( clear_i ),
    .flags_o (         ),
    .push_i  ( data_in ),
    .pop_o   ( data_out)
  );

endmodule