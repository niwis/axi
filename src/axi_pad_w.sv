// Copyright (c) 2014-2018 ETH Zurich, University of Bologna
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Nils Wistoff <nwistoff@ethz.ch>

import axi_pkg::*;


/// Multiple AXI4 cuts.
///
/// These can be used to relax timing pressure on very long AXI busses.
module axi_pad_w #(
  /// The address width.
  parameter int unsigned AddrWidth = 0,
  /// The data width.
  parameter int unsigned DataWidth = 0,
  /// The ID width.
  parameter int unsigned IDWidth   = 0,
  // The user data width.
  parameter int unsigned UserWidth = 0,
  // The number of cycles to pad to.
  parameter int unsigned PadCycles = 0,
  // Choose whether to use dynamic padding. This increases the number of pad
  // cycles if a transaction took longer than PadCycles. Activate this only for
  // homogeneous traffic (i.e. constant burst length).
  parameter logic        PadDynamic = 0,
  // Default types
  parameter type         resp_t    = logic,
  parameter type         req_t     = logic
)(
  input logic     clk_i  ,
  input logic     rst_ni ,

  input  ariane_axi::req_slv_t  slv_req_i,
  output ariane_axi::resp_slv_t slv_resp_o,
  output ariane_axi::req_slv_t  mst_req_o,
  input  ariane_axi::resp_slv_t mst_resp_i,

  input  logic [31:0]           pad_cycles_i,
  output logic [31:0]           pad_cycles_o
);

  // controller FSM
  typedef enum logic[1:0] {IDLE, BUSY_AW, BUSY_W} state_e;
  state_e state_d, state_q;

  logic [31:0] counter_d, counter_q, pad_cycles_d, pad_cycles_q;

  always_comb begin : fsm
    // Defaults
    state_d             = state_q;
    counter_d           = counter_q + 1;
    pad_cycles_d        = pad_cycles_q;

    slv_resp_o.aw_ready = mst_resp_i.aw_ready;
    slv_resp_o.b_valid  = mst_resp_i.b_valid;
    slv_resp_o.w_ready  = mst_resp_i.w_ready;

    mst_req_o.aw_valid  = slv_req_i.aw_valid;
    mst_req_o.b_ready   = slv_req_i.b_ready;
    mst_req_o.w_valid   = slv_req_i.w_valid;

    unique case (state_q)
      // IDLE: Pass everything.
      IDLE: begin
        if (mst_resp_i.b_valid) begin
          counter_d           = '0;
          if (counter_q > pad_cycles_q && PadDynamic)
            pad_cycles_d = counter_q;
        end
        
        if (counter_q == '0)
          counter_d = '0;

        slv_resp_o.aw_ready = 1'b0;
        mst_req_o.aw_valid  = 1'b0;
        
        // We have an incoming read request
        if (slv_req_i.aw_valid)
          state_d = BUSY_AW;
      end

      // BUSY_AW: Pass everything. Start the counter.
      BUSY_AW: begin
        counter_d = counter_q + 1;

        slv_resp_o.w_ready  = 1'b0;
        mst_req_o.w_valid   = 1'b0;
        // The AR transfer is complete, ar_valid is feed through and we are here because it is valid
        if (mst_resp_i.aw_ready)
          state_d = BUSY_W;

      end

      // BUSY_R: Block incoming AR requests.
      BUSY_W: begin
        counter_d           = counter_q + 1;
        slv_resp_o.aw_ready = 1'b0;
        mst_req_o.aw_valid  = 1'b0;
        // We have reached the target pad time.
        if (counter_q + 1 >= pad_cycles_q)
          state_d = IDLE;
        // We have not reached the target pad time yet and this is the last package. Deassert the handshake signals.
        if (mst_resp_i.b_valid) begin
          slv_resp_o.b_valid = 1'b0;
          mst_req_o.b_ready  = 1'b0;
        end
      end
      default: /*not used*/ ;
    endcase

    // Overwrite pad_cycles
    if (pad_cycles_i != 32'hffffffff)
      pad_cycles_d = pad_cycles_i;

  end

  assign mst_req_o.aw        = slv_req_i.aw;
  assign mst_req_o.ar_valid  = slv_req_i.ar_valid;
  assign mst_req_o.w         = slv_req_i.w;
  assign mst_req_o.ar        = slv_req_i.ar;
  assign mst_req_o.r_ready   = slv_req_i.r_ready;

  assign slv_resp_o.r_valid  = mst_resp_i.r_valid;
  assign slv_resp_o.ar_ready = mst_resp_i.ar_ready;
  assign slv_resp_o.b        = mst_resp_i.b;
  assign slv_resp_o.r        = mst_resp_i.r;

  assign pad_cycles_o = pad_cycles_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin : p_regs
    if(!rst_ni) begin
      state_q          <= IDLE;
      counter_q        <= '0;
      pad_cycles_q     <= PadCycles;
    end else begin
      state_q          <= state_d;
      counter_q        <= counter_d;
      pad_cycles_q     <= pad_cycles_d;
    end
  end

endmodule


module axi_pad_w_intf #(
  /// The address width.
  parameter int unsigned ADDR_WIDTH = 0,
  /// The data width.
  parameter int unsigned DATA_WIDTH = 0,
  /// The ID width.
  parameter int unsigned ID_WIDTH   = 0,
  // The user data width.
  parameter int unsigned USER_WIDTH = 0,
  // The number of cycles to pad to.
  parameter int unsigned PAD_CYCLES = 32,
  // Choose whether to use dynamic padding. This increases the number of pad
  // cycles if a transaction took longer than PadCycles. Activate this only for
  // homogeneous traffic (i.e. constant burst length).
  parameter logic        PAD_DYNAMIC= 0
)(
  input logic     clk_i  ,
  input logic     rst_ni ,
  AXI_BUS.Slave   slv     ,
  AXI_BUS.Master  mst     ,
  input  logic [31:0]           pad_cycles_i,
  output logic [31:0]           pad_cycles_o
);

  ariane_axi::req_slv_t  slv_req,  mst_req;
  ariane_axi::resp_slv_t slv_resp, mst_resp;

  axi_pad_w #(
    .AddrWidth ( ADDR_WIDTH             ),
    .DataWidth ( DATA_WIDTH             ),
    .IDWidth   ( ID_WIDTH               ),
    .UserWidth ( USER_WIDTH             ),
    .PadCycles ( PAD_CYCLES             ),
    .PadDynamic( PAD_DYNAMIC            ),
    .req_t     ( ariane_axi::req_slv_t  ),
    .resp_t    ( ariane_axi::resp_slv_t )
  ) i_axi_pad_w (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req  ),
    .slv_resp_o ( slv_resp ),
    .mst_req_o  ( mst_req  ),
    .mst_resp_i ( mst_resp ),
    .pad_cycles_i,
    .pad_cycles_o
  );

  // Interface assigns
  assign slv_req.aw.id     = slv.aw_id;
  assign slv_req.aw.addr   = slv.aw_addr;
  assign slv_req.aw.len    = slv.aw_len;
  assign slv_req.aw.size   = slv.aw_size;
  assign slv_req.aw.burst  = slv.aw_burst;
  assign slv_req.aw.lock   = slv.aw_lock;
  assign slv_req.aw.cache  = slv.aw_cache;
  assign slv_req.aw.prot   = slv.aw_prot;
  assign slv_req.aw.qos    = slv.aw_qos;
  assign slv_req.aw.region = slv.aw_region;
  assign slv_req.aw.atop   = slv.aw_atop;
  // assign slv_req.aw.user   = slv.aw_user;
  assign slv_req.aw_valid  = slv.aw_valid;
  assign slv_req.w.data    = slv.w_data;
  assign slv_req.w.strb    = slv.w_strb;
  assign slv_req.w.last    = slv.w_last;
  // assign slv_req.w.user    = slv.w_user;
  assign slv_req.w_valid   = slv.w_valid;
  assign slv_req.b_ready   = slv.b_ready;
  assign slv_req.ar.id     = slv.ar_id;
  assign slv_req.ar.addr   = slv.ar_addr;
  assign slv_req.ar.len    = slv.ar_len;
  assign slv_req.ar.size   = slv.ar_size;
  assign slv_req.ar.burst  = slv.ar_burst;
  assign slv_req.ar.lock   = slv.ar_lock;
  assign slv_req.ar.cache  = slv.ar_cache;
  assign slv_req.ar.prot   = slv.ar_prot;
  assign slv_req.ar.qos    = slv.ar_qos;
  assign slv_req.ar.region = slv.ar_region;
  // assign slv_req.ar.user   = slv.ar_user;
  assign slv_req.ar_valid  = slv.ar_valid;
  assign slv_req.r_ready   = slv.r_ready;

  assign slv.aw_ready      = slv_resp.aw_ready;
  assign slv.w_ready       = slv_resp.w_ready;
  assign slv.b_id          = slv_resp.b.id;
  assign slv.b_resp        = slv_resp.b.resp;
  assign slv.b_user        = '0;
  assign slv.b_valid       = slv_resp.b_valid;
  assign slv.ar_ready      = slv_resp.ar_ready;
  assign slv.r_id          = slv_resp.r.id;
  assign slv.r_data        = slv_resp.r.data;
  assign slv.r_resp        = slv_resp.r.resp;
  assign slv.r_last        = slv_resp.r.last;
  assign slv.r_user        = '0;
  assign slv.r_valid       = slv_resp.r_valid;

  assign mst.aw_id     = mst_req.aw.id;
  assign mst.aw_addr   = mst_req.aw.addr;
  assign mst.aw_len    = mst_req.aw.len;
  assign mst.aw_size   = mst_req.aw.size;
  assign mst.aw_burst  = mst_req.aw.burst;
  assign mst.aw_lock   = mst_req.aw.lock;
  assign mst.aw_cache  = mst_req.aw.cache;
  assign mst.aw_prot   = mst_req.aw.prot;
  assign mst.aw_qos    = mst_req.aw.qos;
  assign mst.aw_region = mst_req.aw.region;
  assign mst.aw_atop   = mst_req.aw.atop;
  assign mst.aw_user   = '0;
  assign mst.aw_valid  = mst_req.aw_valid;
  assign mst.w_data    = mst_req.w.data;
  assign mst.w_strb    = mst_req.w.strb;
  assign mst.w_last    = mst_req.w.last;
  assign mst.w_user    = '0;
  assign mst.w_valid   = mst_req.w_valid;
  assign mst.b_ready   = mst_req.b_ready;
  assign mst.ar_id     = mst_req.ar.id;
  assign mst.ar_addr   = mst_req.ar.addr;
  assign mst.ar_len    = mst_req.ar.len;
  assign mst.ar_size   = mst_req.ar.size;
  assign mst.ar_burst  = mst_req.ar.burst;
  assign mst.ar_lock   = mst_req.ar.lock;
  assign mst.ar_cache  = mst_req.ar.cache;
  assign mst.ar_prot   = mst_req.ar.prot;
  assign mst.ar_qos    = mst_req.ar.qos;
  assign mst.ar_region = mst_req.ar.region;
  assign mst.ar_user   = '0;
  assign mst.ar_valid  = mst_req.ar_valid;
  assign mst.r_ready   = mst_req.r_ready;

  assign mst_resp.aw_ready = mst.aw_ready;
  assign mst_resp.w_ready  = mst.w_ready;
  assign mst_resp.b.id     = mst.b_id;
  assign mst_resp.b.resp   = mst.b_resp;
  // assign mst_resp.b.user   = mst.b_user;
  assign mst_resp.b_valid  = mst.b_valid;
  assign mst_resp.ar_ready = mst.ar_ready;
  assign mst_resp.r.id     = mst.r_id;
  assign mst_resp.r.data   = mst.r_data;
  assign mst_resp.r.resp   = mst.r_resp;
  assign mst_resp.r.last   = mst.r_last;
  // assign mst_resp.r.user   = mst.r_user;
  assign mst_resp.r_valid  = mst.r_valid;

endmodule