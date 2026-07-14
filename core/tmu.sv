// Copyright 2026 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Description: Trusted Memory Unit (TMU) - load address check
// Placeholder for a future multi-cycle TMU. The check starts in parallel with the
// MMU/PMP translation when the load unit requests translation and operates on the
// virtual address. tmu_done may take an arbitrary number of cycles.

`include "common_cells/registers.svh"

module tmu
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type dcache_req_i_t = logic,
    parameter type dcache_req_o_t = logic,
    parameter type exception_t = logic
) (
    // Subsystem Clock - SUBSYSTEM
    input  logic                     clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input  logic                     rst_ni,
    // Flush signal - CONTROLLER
    input  logic                     flush_i,
    // Load translation request - LOAD_UNIT
    input  logic                     ld_translation_req_i,
    // Load virtual address - LOAD_UNIT
    input  logic [CVA6Cfg.VLEN-1:0]  ld_vaddr_i,
    // Transformed trap instruction - LOAD_UNIT
    input  logic [31:0]              ld_tinst_i,
    // Virtualization mode for load/store - CSR_REGFILE
    input  logic                     ld_st_v_i,
    // Kill in-flight TMU check - LOAD_UNIT
    input  logic                     tmu_kill_i,
    // Enable virtual memory translation for load/stores - CSR_REGFILE
    input  logic                     en_ld_st_translation_i,
    // Enable G-Stage memory translation for load/stores - CSR_REGFILE
    input  logic                     en_ld_st_g_translation_i,
    // Data cache response - CACHES
    input  dcache_req_o_t            req_port_i,
    // Data cache request - CACHES
    output dcache_req_i_t            req_port_o,
    // TMU address check complete - LOAD_UNIT
    output logic                     tmu_hit_o,
    // TMU exception - LOAD_UNIT
    output exception_t               tmu_exception_o
);

// `define TEST_FAULT

  localparam int unsigned TMU_CHECK_DELAY_CYCLES = 9;
  localparam int unsigned TMU_DELAY_CNT_W = (TMU_CHECK_DELAY_CYCLES <= 1) ? 1 : $clog2(
      TMU_CHECK_DELAY_CYCLES
  );

  localparam int unsigned ELEMENT_SIZE = 4;
  localparam int unsigned MATRIX_WIDTH = 48;
  localparam int unsigned MATRIX_HEIGHT = 4*48;
  localparam int unsigned TILE_WIDTH = 16;
  localparam int unsigned TILE_HEIGHT = 16;
  localparam logic [CVA6Cfg.VLEN-1:0] MATRIX_ADDRESS = 32'h9000_0000;
  localparam logic [CVA6Cfg.VLEN-1:0] tile_table_root = 32'h8100_0000;

  // TMU TLB parameters and structs
  localparam int unsigned TMU_N_TLB_ENTRIES = 4;
  localparam int unsigned TMU_INDEX_WIDTH = $clog2(TMU_N_TLB_ENTRIES);

  typedef struct packed {
    logic [CVA6Cfg.VLEN-1:0] addr_low;
    logic [CVA6Cfg.VLEN-1:0] addr_high;
    logic                    valid;
  } tmu_tlb_tag_t;

  typedef struct packed {
    logic tile_valid;
  } tmu_tlb_content_t;
  // Byte alignment
  localparam int unsigned TILE_ENTRY_SIZE = ((($bits(tmu_tlb_content_t)) + (8 - 1)) & ~(8 - 1));

  logic [CVA6Cfg.VLEN-1:0] tile_offset, tile_x, tile_y, tile_entry_addr;

  // TMU TLB registers
  tmu_tlb_tag_t [TMU_N_TLB_ENTRIES-1:0] tmu_tlb_tags_q, tmu_tlb_tags_d;
  tmu_tlb_content_t [TMU_N_TLB_ENTRIES-1:0] tmu_tlb_contents_q, tmu_tlb_contents_d;

  `FF(tmu_tlb_tags_q, tmu_tlb_tags_d, '0);
  `FF(tmu_tlb_contents_q, tmu_tlb_contents_d, '0);

  // TMU TLB tag matching
  logic [TMU_N_TLB_ENTRIES-1:0] tmu_tlb_hits;
  logic [TMU_INDEX_WIDTH-1:0] tmu_tlb_hit_index;
  logic tmu_tlb_hit;

  for (genvar i = 0; i < TMU_N_TLB_ENTRIES; i++) begin : gen_tmu_tlb_tag_match
    always_comb begin
      tmu_tlb_hits[i] = 1'b0;
      if (tmu_tlb_tags_q[i].valid && tmu_tlb_tags_q[i].addr_low <= ld_vaddr_i && ld_vaddr_i < tmu_tlb_tags_q[i].addr_high) begin
        tmu_tlb_hits[i] = 1'b1;
      end
    end
  end

  always_comb begin
    tmu_tlb_hit_index = '0;
    for (int i = 0; i < TMU_N_TLB_ENTRIES; i++) begin
      if (tmu_tlb_hits[i]) begin
        tmu_tlb_hit_index = i[TMU_INDEX_WIDTH-1:0];
        break;
      end
    end
  end

  assign tmu_tlb_hit = |tmu_tlb_hits;
  // When VM is off for load/store, bypass the TMU the same way the MMU forces a DTLB hit.
  // Only apply TMU checks to addresses within the matrix.
  logic tmu_en;
  assign tmu_en = (en_ld_st_translation_i || en_ld_st_g_translation_i)
      && (ld_vaddr_i >= MATRIX_ADDRESS)
      && (ld_vaddr_i < MATRIX_ADDRESS + MATRIX_WIDTH * MATRIX_HEIGHT * ELEMENT_SIZE);
  assign tmu_hit_o = tmu_en ? tmu_tlb_hit : 1'b1;

  always_comb begin
    tmu_exception_o = '0;
    if (tmu_en && tmu_tlb_hit && ~tmu_tlb_contents_q[tmu_tlb_hit_index].tile_valid) begin
      tmu_exception_o.valid = 1'b1;
      tmu_exception_o.cause = riscv::CUSTOM_USER_TRAP;
    end
  end

  // TMU TLB replacement
  logic [TMU_INDEX_WIDTH-1:0] tmu_tlb_replace_index_d, tmu_tlb_replace_index_q;
  `FF(tmu_tlb_replace_index_q, tmu_tlb_replace_index_d, '0);

  // TMU Tile Table Walker
  enum logic [2:0] {
    IDLE,
    WALK,
    WAIT_GNT,
    SEND_TAG,
    WAIT_RVALID
  }
      tmu_ttw_state_d, tmu_ttw_state_q;
  `FF(tmu_ttw_state_q, tmu_ttw_state_d, IDLE);

  logic [TMU_DELAY_CNT_W-1:0] tmu_ttw_delay_cnt_d, tmu_ttw_delay_cnt_q;
  `FF(tmu_ttw_delay_cnt_q, tmu_ttw_delay_cnt_d, '0);

  assign req_port_o.address_index = tile_entry_addr[CVA6Cfg.DCACHE_INDEX_WIDTH-1:0];
  assign req_port_o.address_tag = tile_entry_addr[
      CVA6Cfg.DCACHE_INDEX_WIDTH+CVA6Cfg.DCACHE_TAG_WIDTH-1:CVA6Cfg.DCACHE_INDEX_WIDTH
  ];
  assign req_port_o.data_wdata = '0;
  assign req_port_o.data_wuser = '0;
  assign req_port_o.data_id = '0;

  always_comb begin
    tmu_ttw_state_d         = tmu_ttw_state_q;
    tmu_ttw_delay_cnt_d     = tmu_ttw_delay_cnt_q;
    tmu_tlb_tags_d          = tmu_tlb_tags_q;
    tmu_tlb_contents_d      = tmu_tlb_contents_q;
    tmu_tlb_replace_index_d = tmu_tlb_replace_index_q;
    req_port_o.data_req     = 1'b0;
    req_port_o.data_we      = 1'b0;
    req_port_o.data_be      = '1;
    req_port_o.data_size    = $clog2(TILE_ENTRY_SIZE);
    req_port_o.kill_req     = 1'b0;
    req_port_o.tag_valid    = 1'b0;
    req_port_o.cbo_op       = ariane_pkg::CBO_NONE;

    // Abort an in-flight walk when the load unit kills the check (e.g. MMU fault
    // while waiting in WAIT_TMU). Do not install a TLB entry for the cancelled request.
    if (tmu_kill_i) begin
      tmu_ttw_state_d     = IDLE;
      tmu_ttw_delay_cnt_d = '0;
    end else begin
      case (tmu_ttw_state_q)
        IDLE: begin
          if (tmu_en && ld_translation_req_i && !tmu_tlb_hit) begin
            tmu_ttw_state_d = WALK;
          end
`ifdef TEST_FAULT
          if ( tmu_tlb_hits[3] ) begin
            tmu_tlb_tags_d[3] = '0;
          end
`endif
        end
        WALK: begin
          // Model the latency required to calculate the tile-table entry address.
          if (tmu_ttw_delay_cnt_q == TMU_CHECK_DELAY_CYCLES - 1) begin
            tmu_ttw_delay_cnt_d = '0;
            tmu_ttw_state_d = WAIT_GNT;
          end else begin
            tmu_ttw_delay_cnt_d = tmu_ttw_delay_cnt_q + 1'b1;
          end
        end
        WAIT_GNT: begin
          req_port_o.data_req = 1'b1;
          if (req_port_i.data_gnt) begin
            tmu_ttw_state_d = SEND_TAG;
          end
        end
        SEND_TAG: begin
          req_port_o.tag_valid = 1'b1;
          if (req_port_i.data_rvalid) begin
            tmu_ttw_state_d = IDLE;
          end else begin
            tmu_ttw_state_d = WAIT_RVALID;
          end
        end
        WAIT_RVALID: begin
          if (req_port_i.data_rvalid) begin
            tmu_ttw_state_d = IDLE;
          end
        end
        default: tmu_ttw_state_d = IDLE;
      endcase

      if (req_port_i.data_rvalid
          && (tmu_ttw_state_q == SEND_TAG || tmu_ttw_state_q == WAIT_RVALID)) begin
        // Replace the TLB entry whether the response arrives with tag_valid or later.
        tmu_tlb_tags_d[tmu_tlb_replace_index_q] = '{
            valid: 1'b1,
            addr_low: ld_vaddr_i,
            addr_high: ld_vaddr_i + TILE_WIDTH * ELEMENT_SIZE
        };
        tmu_tlb_contents_d[tmu_tlb_replace_index_q] =
            tmu_tlb_content_t'(req_port_i.data_rdata);
        // Round robin update index
        tmu_tlb_replace_index_d = tmu_tlb_replace_index_q + 1'b1;
        if (tmu_tlb_replace_index_q == TMU_N_TLB_ENTRIES - 1) begin
          tmu_tlb_replace_index_d = '0;
        end
      end
    end
  end

  // Tile offset calculation
  always_comb begin
    tile_offset = ld_vaddr_i - MATRIX_ADDRESS;
    tile_y = tile_offset / (MATRIX_WIDTH * TILE_HEIGHT * ELEMENT_SIZE);
    tile_x = (tile_offset % (MATRIX_WIDTH * ELEMENT_SIZE)) / (TILE_WIDTH * ELEMENT_SIZE);
    tile_entry_addr = tile_table_root
        + ((tile_y * (MATRIX_WIDTH / TILE_WIDTH) + tile_x) * TILE_ENTRY_SIZE);
  end

`ifndef SYNTHESIS
  // Capture the VA when a walk starts and check it does not change mid-walk.
  // Functional logic currently uses live ld_vaddr_i; this only validates the
  // assumed handshake from the load unit.
  logic [CVA6Cfg.VLEN-1:0] ld_vaddr_walk_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ld_vaddr_walk_q <= '0;
    end else if (tmu_ttw_state_q == IDLE && tmu_ttw_state_d == WALK) begin
      ld_vaddr_walk_q <= ld_vaddr_i;
    end
  end

  tmu_vaddr_stable_during_walk :
  assert property (@(posedge clk_i) disable iff (~rst_ni)
      (tmu_ttw_state_q != IDLE) |-> (ld_vaddr_i == ld_vaddr_walk_q))
  else $error("TMU: ld_vaddr_i changed mid-walk");
`endif

endmodule
