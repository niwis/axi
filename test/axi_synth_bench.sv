// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Authors:
// - Wolfgang Roenninger <wroennin@iis.ee.ethz.ch>
// - Andreas Kurth <akurth@iis.ee.ethz.ch>
// - Fabian Schuiki <fschuiki@iis.ee.ethz.ch>
// - Michael Rogenmoser <michaero@iis.ee.ethz.ch>

/// A synthesis test bench which instantiates various adapter variants.
module axi_synth_bench (
  input logic clk_i,
  input logic rst_ni
);

  localparam int AXI_ADDR_WIDTH[6] = '{32, 64, 1, 2, 42, 129};
  localparam int AXI_ID_USER_WIDTH[3] = '{0, 1, 8};
  localparam int NUM_SLAVE_MASTER[3] = '{1, 2, 4};

  // AXI_DATA_WIDTH = {8, 16, 32, 64, 128, 256, 512, 1024}
  for (genvar i = 0; i < 8; i++) begin
    localparam DW = (2**i) * 8;
    synth_slice #(.AW(32), .DW(DW), .IW(8), .UW(8)) s(.*);
  end

  // AXI_ADDR_WIDTH
  for (genvar i = 0; i < 6; i++) begin
    localparam int AW = AXI_ADDR_WIDTH[i];
    synth_slice #(.AW(AW), .DW(32), .IW(8), .UW(8)) s(.*);
  end

  // AXI_ID_WIDTH and AXI_USER_WIDTH
  for (genvar i = 0; i < 3; i++) begin
    localparam int UW = AXI_ID_USER_WIDTH[i];
    localparam int IW = (UW == 0) ? 1 : UW;
    synth_slice #(.AW(32), .DW(32), .IW(IW), .UW(UW)) s(.*);
  end

  // ATOP Filter
  for (genvar iID = 1; iID <= 8; iID++) begin
    localparam int IW = iID;
    for (genvar iTxn = 1; iTxn <= 12; iTxn++) begin
      localparam int WT = iTxn;
      synth_axi_atop_filter #(
        .AXI_ADDR_WIDTH     (64),
        .AXI_DATA_WIDTH     (64),
        .AXI_ID_WIDTH       (IW),
        .AXI_USER_WIDTH     (4),
        .AXI_MAX_WRITE_TXNS (WT)
      ) i_filter (.*);
    end
  end

  // AXI4-Lite crossbar
  for (genvar i = 0; i < 3; i++) begin
    synth_axi_lite_xbar #(
      .NoSlvMst  ( NUM_SLAVE_MASTER[i] )
    ) i_lite_xbar (.*);
  end

  // Clock Domain Crossing
  for (genvar i = 0; i < 6; i++) begin
    localparam int AW = AXI_ADDR_WIDTH[i];
    for (genvar j = 0; j < 3; j++) begin
      localparam IUW = AXI_ID_USER_WIDTH[j];
      synth_axi_cdc #(
        .AXI_ADDR_WIDTH (AW),
        .AXI_DATA_WIDTH (128),
        .AXI_ID_WIDTH   (IUW),
        .AXI_USER_WIDTH (IUW)
      ) i_cdc (.*);
    end
  end

  // AXI4-Lite to APB bridge
  for (genvar i_data = 0; i_data < 3; i_data++) begin
    localparam int unsigned DataWidth = (2**i_data) * 8;
    for (genvar i_slv = 0; i_slv < 3; i_slv++) begin
      synth_axi_lite_to_apb #(
        .NoApbSlaves ( NUM_SLAVE_MASTER[i_slv] ),
        .DataWidth   ( DataWidth               )
      ) i_axi_lite_to_apb (.*);
    end
  end

  // AXI4-Lite Mailbox
  for (genvar i_irq_mode = 0; i_irq_mode < 4; i_irq_mode++) begin
    localparam bit EDGE_TRIG = i_irq_mode[0];
    localparam bit ACT_HIGH  = i_irq_mode[1];
    for (genvar i_depth = 2; i_depth < 8; i_depth++) begin
      localparam int unsigned DEPTH = 2**i_depth;
      synth_axi_lite_mailbox #(
        .MAILBOX_DEPTH ( DEPTH     ),
        .IRQ_EDGE_TRIG ( EDGE_TRIG ),
        .IRQ_ACT_HIGH  ( ACT_HIGH  )
      ) i_axi_lite_mailbox (.*);
    end
  end

  // AXI Isolation module
  for (genvar i = 0; i < 6; i++) begin
    synth_axi_isolate #(
      .NumPending   ( AXI_ADDR_WIDTH[i] ),
      .AxiIdWidth   ( 32'd10            ),
      .AxiAddrWidth ( 32'd64            ),
      .AxiDataWidth ( 32'd512           ),
      .AxiUserWidth ( 32'd10            )
    ) i_synth_axi_isolate (.*);
  end

  for (genvar i = 0; i < 6; i++) begin
    localparam int unsigned SLV_PORT_ADDR_WIDTH = AXI_ADDR_WIDTH[i];
    if (SLV_PORT_ADDR_WIDTH > 12) begin
      for (genvar j = 0; j < 6; j++) begin
        localparam int unsigned MST_PORT_ADDR_WIDTH = AXI_ADDR_WIDTH[j];
        if (MST_PORT_ADDR_WIDTH > 12) begin
          synth_axi_modify_address #(
            .AXI_SLV_PORT_ADDR_WIDTH  (SLV_PORT_ADDR_WIDTH),
            .AXI_MST_PORT_ADDR_WIDTH  (MST_PORT_ADDR_WIDTH),
            .AXI_DATA_WIDTH           (128),
            .AXI_ID_WIDTH             (5),
            .AXI_USER_WIDTH           (2)
          ) i_synth_axi_modify_address ();
        end
      end
    end
  end

  // AXI4+ATOP serializer
  for (genvar i = 0; i < 6; i++) begin
    synth_axi_serializer #(
      .NumPending   ( AXI_ADDR_WIDTH[i] ),
      .AxiIdWidth   ( 32'd10            ),
      .AxiAddrWidth ( 32'd64            ),
      .AxiDataWidth ( 32'd512           ),
      .AxiUserWidth ( 32'd10            )
    ) i_synth_axi_serializer (.*);
  end

  // AXI4-Lite Registers
  for (genvar i = 0; i < 6; i++) begin
    localparam int unsigned NUM_BYTES[6] = '{1, 4, 42, 64, 129, 512};
    synth_axi_lite_regs #(
      .REG_NUM_BYTES  ( NUM_BYTES[i]      ),
      .AXI_ADDR_WIDTH ( 32'd32            ),
      .AXI_DATA_WIDTH ( 32'd32            )
    ) i_axi_lite_regs (.*);
  end

  // AXI ID width converter
  for (genvar i_iwus = 0; i_iwus < 3; i_iwus++) begin : gen_iw_upstream
    localparam int unsigned AxiIdWidthUs = AXI_ID_USER_WIDTH[i_iwus] + 1;
    for (genvar i_iwds = 0; i_iwds < 3; i_iwds++) begin : gen_iw_downstream
      localparam int unsigned AxiIdWidthDs = AXI_ID_USER_WIDTH[i_iwds] + 1;
      localparam int unsigned TableSize    = 2**AxiIdWidthDs;
      synth_axi_iw_converter # (
        .AxiSlvPortIdWidth      ( AxiIdWidthUs    ),
        .AxiMstPortIdWidth      ( AxiIdWidthDs    ),
        .AxiSlvPortMaxUniqIds   ( 2**AxiIdWidthUs ),
        .AxiSlvPortMaxTxnsPerId ( 13              ),
        .AxiSlvPortMaxTxns      ( 81              ),
        .AxiMstPortMaxUniqIds   ( 2**AxiIdWidthDs ),
        .AxiMstPortMaxTxnsPerId ( 11              ),
        .AxiAddrWidth           ( 32'd64          ),
        .AxiDataWidth           ( 32'd512         ),
        .AxiUserWidth           ( 32'd10          )
      ) i_synth_axi_iw_converter (.*);
    end
  end

  // AXI4+ATOP on chip memory slave banked
  for (genvar i = 0; i < 5; i++) begin : gen_axi_to_mem_banked_data
    for (genvar j = 0; j < 4; j++) begin : gen_axi_to_mem_banked_bank_num
      for (genvar k = 0; k < 2; k++) begin : gen_axi_to_mem_banked_bank_addr
        localparam int unsigned DATA_WIDTH_AXI[5]   = {32'd32, 32'd64, 32'd128, 32'd256, 32'd512};
        localparam int unsigned NUM_BANKS[4]        = {32'd2,  32'd4,  32'd6,   32'd8};
        localparam int unsigned ADDR_WIDTH_BANKS[2] = {32'd5,  32'd11};

        synth_axi_to_mem_banked #(
          .AxiDataWidth  ( DATA_WIDTH_AXI[i]   ),
          .BankNum       ( NUM_BANKS[j]        ),
          .BankAddrWidth ( ADDR_WIDTH_BANKS[k] )
        ) i_axi_to_mem_banked (.*);
      end
    end
  end

  // AXI4-Lite DW converter
  for (genvar i = 0; i < 3; i++) begin
    for (genvar j = 0; j < 3; j++) begin
      localparam int unsigned SLV_DW[3] = {32, 64, 128};
      localparam int unsigned MST_DW[3] = {16, 32, 64};

      synth_axi_lite_dw_converter #(
        .AXI_SLV_PORT_DATA_WIDTH (SLV_DW[i]),
        .AXI_MST_PORT_DATA_WIDTH (MST_DW[j])
      ) i_axi_lite_dw_converter (.*);
    end
  end

  for (genvar iReadTxns = 1; iReadTxns <= 4; iReadTxns++) begin
    for (genvar iWriteTxns = 1; iWriteTxns <= 4; iWriteTxns++) begin
      for (genvar iFullBW = 0; iFullBW <= 1; iFullBW++) begin
        synth_axi_burst_splitter #(
          .MaxReadTxns  (iReadTxns),
          .MaxWriteTxns (iWriteTxns),
          .FullBW       (iFullBW),
          .AddrWidth    (64),
          .DataWidth    (64),
          .IdWidth      (8),
          .UserWidth    (8)
        ) i_splitter (.*);
      end
    end
  end

  // AXI Burst Splitter (granular)
  for (genvar iReadTxns = 1; iReadTxns <= 4; iReadTxns++) begin : gen_burst_splitter_gran_rd
    for (genvar iWriteTxns = 1; iWriteTxns <= 4; iWriteTxns++) begin : gen_burst_splitter_gran_wr
      for (genvar iFullBW = 0; iFullBW <= 1; iFullBW++) begin : gen_burst_splitter_gran_bw
        synth_axi_burst_splitter_gran #(
          .MaxReadTxns  ( iReadTxns ),
          .MaxWriteTxns ( iWriteTxns ),
          .FullBW       ( iFullBW ),
          .AddrWidth    ( 64 ),
          .DataWidth    ( 64 ),
          .IdWidth      ( 8 ),
          .UserWidth    ( 8 )
        ) i_splitter_gran (.*);
      end
    end
  end

  // AXI Bus Compare
  for (genvar iFifoDepth = 4; iFifoDepth <= 16; iFifoDepth += 12) begin : gen_bus_compare
    for (genvar iUseSize = 0; iUseSize <= 1; iUseSize++) begin : gen_bus_compare_size
      synth_axi_bus_compare #(
        .FifoDepth ( iFifoDepth ),
        .UseSize   ( iUseSize   )
      ) i_bus_compare (.*);
    end
  end

  // AXI Crossbar
  for (genvar i = 0; i < 3; i++) begin : gen_axi_xbar
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_xbar #(
      .NoSlvPorts ( NoPorts[i] ),
      .NoMstPorts ( NoPorts[i] )
    ) i_axi_xbar (.*);
  end

  // AXI Register Slice
  for (genvar i = 0; i < 2; i++) begin : gen_axi_cut
    synth_axi_cut #(
      .Bypass ( i[0] )
    ) i_axi_cut (.*);
  end

  // AXI Multi-cut
  for (genvar i = 0; i < 3; i++) begin : gen_axi_multicut
    localparam int unsigned NoCuts[3] = '{1, 2, 4};
    synth_axi_multicut #(
      .NoCuts ( NoCuts[i] )
    ) i_axi_multicut (.*);
  end

  // AXI FIFO
  for (genvar i = 0; i < 3; i++) begin : gen_axi_fifo
    localparam int unsigned Depth[3] = '{1, 4, 16};
    synth_axi_fifo #(
      .Depth ( Depth[i] )
    ) i_axi_fifo (.*);
  end

  // AXI Demux
  for (genvar i = 0; i < 3; i++) begin : gen_axi_demux
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_demux #(
      .NoMstPorts ( NoPorts[i] )
    ) i_axi_demux (.*);
  end

  // AXI Demux (simple)
  for (genvar i = 0; i < 3; i++) begin : gen_axi_demux_simple
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_demux_simple #(
      .NoMstPorts ( NoPorts[i] )
    ) i_axi_demux_simple (.*);
  end

  // AXI Mux
  for (genvar i = 0; i < 3; i++) begin : gen_axi_mux
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_mux #(
      .NoSlvPorts ( NoPorts[i] )
    ) i_axi_mux (.*);
  end

  // AXI Data Width Converter
  for (genvar i = 0; i < 3; i++) begin : gen_axi_dw_converter
    localparam int unsigned SlvDW[3] = '{32, 64, 128};
    localparam int unsigned MstDW[3] = '{64, 32,  64};
    synth_axi_dw_converter #(
      .AxiSlvPortDataWidth ( SlvDW[i] ),
      .AxiMstPortDataWidth ( MstDW[i] )
    ) i_axi_dw_converter (.*);
  end

  // AXI Throttle
  for (genvar i = 0; i < 3; i++) begin : gen_axi_throttle
    localparam int unsigned MaxPending[3] = '{1, 4, 16};
    synth_axi_throttle #(
      .MaxNumAwPending ( MaxPending[i] ),
      .MaxNumArPending ( MaxPending[i] )
    ) i_axi_throttle (.*);
  end

  // AXI Invalidation Filter
  for (genvar i = 0; i < 3; i++) begin : gen_axi_inval_filter
    localparam int unsigned MaxTxns[3] = '{1, 4, 16};
    synth_axi_inval_filter #(
      .MaxTxns ( MaxTxns[i] )
    ) i_axi_inval_filter (.*);
  end

  // AXI ID Serializer
  for (genvar i_slv = 0; i_slv < 3; i_slv++) begin : gen_axi_id_serialize_slv
    for (genvar i_mst = 0; i_mst < 2; i_mst++) begin : gen_axi_id_serialize_mst
      localparam int unsigned SlvIW[3] = '{4, 6, 8};
      localparam int unsigned MstIW[2] = '{2, 3};
      synth_axi_id_serialize #(
        .AxiSlvPortIdWidth    ( SlvIW[i_slv]    ),
        .AxiMstPortIdWidth    ( MstIW[i_mst]    ),
        .AxiMstPortMaxUniqIds ( 2**MstIW[i_mst] )
      ) i_axi_id_serialize (.*);
    end
  end

  // AXI ID Remap
  for (genvar i_slv = 0; i_slv < 2; i_slv++) begin : gen_axi_id_remap_slv
    for (genvar i_mst = 0; i_mst < 2; i_mst++) begin : gen_axi_id_remap_mst
      localparam int unsigned SlvIW[2] = '{6, 8};
      localparam int unsigned MstIW[2] = '{3, 4};
      synth_axi_id_remap #(
        .AxiSlvPortIdWidth   ( SlvIW[i_slv] ),
        .AxiMstPortIdWidth   ( MstIW[i_mst] )
      ) i_axi_id_remap (.*);
    end
  end

  // AXI ID Prepend
  for (genvar i = 0; i < 3; i++) begin : gen_axi_id_prepend
    localparam int unsigned NoBus[3] = '{1, 2, 4};
    synth_axi_id_prepend #(
      .NoBus ( NoBus[i] )
    ) i_axi_id_prepend ();
  end

  // AXI Demux ID Counters
  for (genvar i = 0; i < 3; i++) begin : gen_axi_demux_id_counters
    localparam int unsigned AxiIdBits[3] = '{1, 3, 4};
    synth_axi_demux_id_counters #(
      .AxiIdBits ( AxiIdBits[i] )
    ) i_axi_demux_id_counters (.*);
  end

  // AXI to Memory
  for (genvar i = 0; i < 3; i++) begin : gen_axi_to_mem
    localparam int unsigned NumBanks[3] = '{1, 2, 4};
    synth_axi_to_mem #(
      .NumBanks ( NumBanks[i] )
    ) i_axi_to_mem (.*);
  end

  // AXI to Memory (interleaved)
  for (genvar i = 0; i < 3; i++) begin : gen_axi_to_mem_interleaved
    localparam int unsigned NumBanks[3] = '{1, 2, 4};
    synth_axi_to_mem_interleaved #(
      .NumBanks ( NumBanks[i] )
    ) i_axi_to_mem_interleaved (.*);
  end

  // AXI to Memory (split)
  for (genvar i = 0; i < 2; i++) begin : gen_axi_to_mem_split
    localparam int unsigned MemDW[2] = '{32, 64};
    synth_axi_to_mem_split #(
      .MemDataWidth ( MemDW[i] )
    ) i_axi_to_mem_split (.*);
  end

  // AXI to Memory (detailed)
  for (genvar i = 0; i < 3; i++) begin : gen_axi_to_detailed_mem
    localparam int unsigned NumBanks[3] = '{1, 2, 4};
    synth_axi_to_detailed_mem #(
      .NumBanks ( NumBanks[i] )
    ) i_axi_to_detailed_mem (.*);
  end

  // AXI from Memory
  for (genvar i = 0; i < 3; i++) begin : gen_axi_from_mem
    localparam int unsigned MaxReqs[3] = '{1, 4, 16};
    synth_axi_from_mem #(
      .MaxRequests ( MaxReqs[i] )
    ) i_axi_from_mem (.*);
  end

  // AXI-Lite from Memory
  for (genvar i = 0; i < 3; i++) begin : gen_axi_lite_from_mem
    localparam int unsigned MaxReqs[3] = '{1, 4, 16};
    synth_axi_lite_from_mem #(
      .MaxRequests ( MaxReqs[i] )
    ) i_axi_lite_from_mem (.*);
  end

  // AXI Interleaved Crossbar
  for (genvar i = 0; i < 3; i++) begin : gen_axi_interleaved_xbar
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_interleaved_xbar #(
      .NoSlvPorts ( NoPorts[i] ),
      .NoMstPorts ( NoPorts[i] )
    ) i_axi_interleaved_xbar (.*);
  end

  // AXI Crosspoint
  for (genvar i = 0; i < 3; i++) begin : gen_axi_xp
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_xp #(
      .NumSlvPorts ( NoPorts[i] ),
      .NumMstPorts ( NoPorts[i] )
    ) i_axi_xp (.*);
  end

  // AXI Burst Unwrap
  for (genvar i = 0; i < 3; i++) begin : gen_axi_burst_unwrap
    localparam int unsigned MaxTxns[3] = '{1, 4, 16};
    synth_axi_burst_unwrap #(
      .MaxReadTxns  ( MaxTxns[i] ),
      .MaxWriteTxns ( MaxTxns[i] )
    ) i_axi_burst_unwrap (.*);
  end

  // AXI Delayer
  synth_axi_delayer i_axi_delayer (.*);

  // AXI Read/Write Join
  synth_axi_rw_join i_axi_rw_join (.*);

  // AXI Read/Write Split
  synth_axi_rw_split i_axi_rw_split (.*);

  // AXI Zero Memory
  synth_axi_zero_mem i_axi_zero_mem (.*);

  // AXI LFSR
  for (genvar i = 0; i < 2; i++) begin : gen_axi_lfsr
    localparam int unsigned DW[2] = '{32, 64};
    synth_axi_lfsr #(
      .DataWidth ( DW[i] )
    ) i_axi_lfsr (.*);
  end

  // AXI-Lite LFSR
  for (genvar i = 0; i < 2; i++) begin : gen_axi_lite_lfsr
    localparam int unsigned DW[2] = '{32, 64};
    synth_axi_lite_lfsr #(
      .DataWidth ( DW[i] )
    ) i_axi_lite_lfsr (.*);
  end

  // AXI Slave Compare
  for (genvar iFifoDepth = 4; iFifoDepth <= 16; iFifoDepth += 12) begin : gen_slave_compare
    for (genvar iUseSize = 0; iUseSize <= 1; iUseSize++) begin : gen_slave_compare_size
      synth_axi_slave_compare #(
        .FifoDepth ( iFifoDepth ),
        .UseSize   ( iUseSize   )
      ) i_slave_compare (.*);
    end
  end

  // AXI FIFO Delay Dyn
  for (genvar i = 0; i < 3; i++) begin : gen_axi_fifo_delay_dyn
    localparam int unsigned MaxDelay[3] = '{16, 64, 256};
    synth_axi_fifo_delay_dyn #(
      .MaxDelay ( MaxDelay[i] )
    ) i_axi_fifo_delay_dyn (.*);
  end

  // AXI Error Slave
  synth_axi_err_slv i_axi_err_slv (.*);

  // AXI CDC Source
  for (genvar i = 0; i < 3; i++) begin : gen_axi_cdc_src
    localparam int unsigned LogDepth[3] = '{1, 2, 3};
    synth_axi_cdc_src #(
      .LogDepth ( LogDepth[i] )
    ) i_axi_cdc_src (.*);
  end

  // AXI CDC Destination
  for (genvar i = 0; i < 3; i++) begin : gen_axi_cdc_dst
    localparam int unsigned LogDepth[3] = '{1, 2, 3};
    synth_axi_cdc_dst #(
      .LogDepth ( LogDepth[i] )
    ) i_axi_cdc_dst (.*);
  end

  // AXI-Lite Demux
  for (genvar i = 0; i < 3; i++) begin : gen_axi_lite_demux
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_lite_demux #(
      .NoMstPorts ( NoPorts[i] )
    ) i_axi_lite_demux (.*);
  end

  // AXI-Lite Mux
  for (genvar i = 0; i < 3; i++) begin : gen_axi_lite_mux
    localparam int unsigned NoPorts[3] = '{1, 2, 4};
    synth_axi_lite_mux #(
      .NoSlvPorts ( NoPorts[i] )
    ) i_axi_lite_mux (.*);
  end

endmodule

// slang lint_off unassigned-variable
// slang lint_off unused-but-set-variable

module synth_slice #(
  parameter int AW = -1,
  parameter int DW = -1,
  parameter int IW = -1,
  parameter int UW = -1
)(
  input logic clk_i,
  input logic rst_ni
);

  AXI_BUS #(
    .AXI_ADDR_WIDTH(AW),
    .AXI_DATA_WIDTH(DW),
    .AXI_ID_WIDTH(IW),
    .AXI_USER_WIDTH(UW)
  ) a_full(), b_full();

  AXI_LITE #(
    .AXI_ADDR_WIDTH(AW),
    .AXI_DATA_WIDTH(DW)
  ) a_lite(), b_lite();

  axi_to_axi_lite_intf #(
    .AXI_ID_WIDTH       (IW),
    .AXI_ADDR_WIDTH     (AW),
    .AXI_DATA_WIDTH     (DW),
    .AXI_USER_WIDTH     (UW),
    .AXI_MAX_WRITE_TXNS (32'd10),
    .AXI_MAX_READ_TXNS  (32'd10),
    .FALL_THROUGH       (1'b0)
  ) a (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .testmode_i (1'b0),
    .slv        (a_full.Slave),
    .mst        (a_lite.Master)
  );
  axi_lite_to_axi_intf #(
    .AXI_DATA_WIDTH (DW)
  ) b (
    .in   (b_lite.Slave),
    .slv_aw_cache_i ('0),
    .slv_ar_cache_i ('0),
    .out  (b_full.Master)
  );

endmodule


module synth_axi_atop_filter #(
  parameter int unsigned AXI_ADDR_WIDTH = 0,
  parameter int unsigned AXI_DATA_WIDTH = 0,
  parameter int unsigned AXI_ID_WIDTH = 0,
  parameter int unsigned AXI_USER_WIDTH = 0,
  parameter int unsigned AXI_MAX_WRITE_TXNS = 0
) (
  input logic clk_i,
  input logic rst_ni
);

  AXI_BUS #(
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH)
  ) upstream ();

  AXI_BUS #(
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH)
  ) downstream ();

  axi_atop_filter_intf #(
    .AXI_ID_WIDTH       (AXI_ID_WIDTH),
    .AXI_MAX_WRITE_TXNS (AXI_MAX_WRITE_TXNS)
  ) dut (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .slv    (upstream),
    .mst    (downstream)
  );
endmodule

`include "axi/typedef.svh"

module synth_axi_lite_to_apb #(
  parameter int unsigned NoApbSlaves = 0,
  parameter int unsigned DataWidth   = 0
) (
  input logic clk_i,  // Clock
  input logic rst_ni  // Asynchronous reset active low
);

  typedef logic [31:0]            addr_t;
  typedef logic [DataWidth-1:0]   data_t;
  typedef logic [DataWidth/8-1:0] strb_t;

  typedef struct packed {
    addr_t          paddr;   // same as AXI4-Lite
    axi_pkg::prot_t pprot;   // same as AXI4-Lite, specification is the same
    logic           psel;    // one request line per connected APB4 slave
    logic           penable; // enable signal shows second APB4 cycle
    logic           pwrite;  // write enable
    data_t          pwdata;  // write data, comes from W channel
    strb_t          pstrb;   // write strb, comes from W channel
  } apb_req_t;

  typedef struct packed {
    logic  pready;   // slave signals that it is ready
    data_t prdata;   // read data, connects to R channel
    logic  pslverr;  // gets translated into either `axi_pkg::RESP_OK` or `axi_pkg::RESP_SLVERR`
  } apb_resp_t;

  `AXI_LITE_TYPEDEF_AW_CHAN_T(aw_chan_t, addr_t)
  `AXI_LITE_TYPEDEF_W_CHAN_T(w_chan_t, data_t, strb_t)
  `AXI_LITE_TYPEDEF_B_CHAN_T(b_chan_t)
  `AXI_LITE_TYPEDEF_AR_CHAN_T(ar_chan_t, addr_t)
  `AXI_LITE_TYPEDEF_R_CHAN_T(r_chan_t, data_t)
  `AXI_LITE_TYPEDEF_REQ_T(axi_req_t, aw_chan_t, w_chan_t, ar_chan_t)
  `AXI_LITE_TYPEDEF_RESP_T(axi_resp_t, b_chan_t, r_chan_t)

  axi_req_t                    axi_req;
  axi_resp_t                   axi_resp;
  apb_req_t  [NoApbSlaves-1:0] apb_req;
  apb_resp_t [NoApbSlaves-1:0] apb_resp;

  axi_pkg::xbar_rule_32_t [NoApbSlaves-1:0] addr_map;

  axi_lite_to_apb #(
    .NoApbSlaves     ( NoApbSlaves             ),
    .NoRules         ( NoApbSlaves             ),
    .AddrWidth       ( 32'd32                  ),
    .DataWidth       ( DataWidth               ),
    .axi_lite_req_t  ( axi_req_t               ),
    .axi_lite_resp_t ( axi_resp_t              ),
    .apb_req_t       ( apb_req_t               ),
    .apb_resp_t      ( apb_resp_t              ),
    .rule_t          ( axi_pkg::xbar_rule_32_t )
  ) i_axi_lite_to_apb_dut (
    .clk_i           ( clk_i    ),
    .rst_ni          ( rst_ni   ),
    .axi_lite_req_i  ( axi_req  ),
    .axi_lite_resp_o ( axi_resp ),
    .apb_req_o       ( apb_req  ),
    .apb_resp_i      ( apb_resp ),
    .addr_map_i      ( addr_map )
  );

endmodule

module synth_axi_cdc #(
  parameter int unsigned AXI_ADDR_WIDTH = 0,
  parameter int unsigned AXI_DATA_WIDTH = 0,
  parameter int unsigned AXI_ID_WIDTH = 0,
  parameter int unsigned AXI_USER_WIDTH = 0
) (
  input logic clk_i,
  input logic rst_ni
);

  AXI_BUS #(
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH)
  ) upstream ();

  AXI_BUS #(
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH)
  ) downstream ();

  axi_cdc_intf #(
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH),
    .LOG_DEPTH      (2)
  ) dut (
    .src_clk_i  (clk_i),
    .src_rst_ni (rst_ni),
    .src        (upstream),
    .dst_clk_i  (clk_i),
    .dst_rst_ni (rst_ni),
    .dst        (downstream)
  );

endmodule

`include "axi/typedef.svh"

module synth_axi_lite_xbar #(
  parameter int unsigned NoSlvMst = 32'd1
) (
  input logic clk_i,  // Clock
  input logic rst_ni  // Asynchronous reset active low
);
  typedef logic [32'd32-1:0]   addr_t;
  typedef logic [32'd32-1:0]   data_t;
  typedef logic [32'd32/8-1:0] strb_t;

  `AXI_LITE_TYPEDEF_AW_CHAN_T(aw_chan_t, addr_t)
  `AXI_LITE_TYPEDEF_W_CHAN_T(w_chan_t, data_t, strb_t)
  `AXI_LITE_TYPEDEF_B_CHAN_T(b_chan_t)
  `AXI_LITE_TYPEDEF_AR_CHAN_T(ar_chan_t, addr_t)
  `AXI_LITE_TYPEDEF_R_CHAN_T(r_chan_t, data_t)
  `AXI_LITE_TYPEDEF_REQ_T(axi_req_t, aw_chan_t, w_chan_t, ar_chan_t)
  `AXI_LITE_TYPEDEF_RESP_T(axi_resp_t, b_chan_t, r_chan_t)
  localparam axi_pkg::xbar_cfg_t XbarCfg = '{
    NoSlvPorts:         NoSlvMst,
    NoMstPorts:         NoSlvMst,
    MaxMstTrans:        32'd5,
    MaxSlvTrans:        32'd5,
    FallThrough:        1'b1,
    LatencyMode:        axi_pkg::CUT_ALL_PORTS,
    AxiAddrWidth:       32'd32,
    AxiDataWidth:       32'd32,
    NoAddrRules:        NoSlvMst,
    default:            '0
  };

  axi_pkg::xbar_rule_32_t [NoSlvMst-1:0] addr_map;
  logic                                  test;
  axi_req_t               [NoSlvMst-1:0] mst_reqs,  slv_reqs;
  axi_resp_t              [NoSlvMst-1:0] mst_resps, slv_resps;

  axi_lite_xbar #(
    .Cfg        ( XbarCfg                 ),
    .aw_chan_t  (  aw_chan_t              ),
    .w_chan_t   (   w_chan_t              ),
    .b_chan_t   (   b_chan_t              ),
    .ar_chan_t  (  ar_chan_t              ),
    .r_chan_t   (   r_chan_t              ),
    .axi_req_t  (  axi_req_t              ),
    .axi_resp_t ( axi_resp_t              ),
    .rule_t     ( axi_pkg::xbar_rule_32_t )
  ) i_xbar_dut (
    .clk_i                 ( clk_i     ),
    .rst_ni                ( rst_ni    ),
    .test_i                ( test      ),
    .slv_ports_req_i       ( mst_reqs  ),
    .slv_ports_resp_o      ( mst_resps ),
    .mst_ports_req_o       ( slv_reqs  ),
    .mst_ports_resp_i      ( slv_resps ),
    .addr_map_i            ( addr_map  ),
    .en_default_mst_port_i ( '0        ),
    .default_mst_port_i    ( '0        )
  );
endmodule

module synth_axi_lite_mailbox #(
  parameter int unsigned MAILBOX_DEPTH = 32'd1,
  parameter bit          IRQ_EDGE_TRIG = 1'b0,
  parameter bit          IRQ_ACT_HIGH  = 1'b0
) (
  input logic clk_i,  // Clock
  input logic rst_ni  // Asynchronous reset active low
);
  typedef logic [32'd32-1:0]   addr_t;

  AXI_LITE #(
    .AXI_ADDR_WIDTH (32'd32),
    .AXI_DATA_WIDTH (32'd32)
  ) slv [1:0] ();

  logic        test;
  logic  [1:0] irq;
  addr_t [1:0] base_addr;

  axi_lite_mailbox_intf #(
    .MAILBOX_DEPTH  ( MAILBOX_DEPTH  ),
    .IRQ_EDGE_TRIG  ( IRQ_EDGE_TRIG  ),
    .IRQ_ACT_HIGH   ( IRQ_ACT_HIGH   ),
    .AXI_ADDR_WIDTH ( 32'd32         ),
    .AXI_DATA_WIDTH ( 32'd32         )
  ) i_axi_lite_mailbox (
    .clk_i       ( clk_i     ), // Clock
    .rst_ni      ( rst_ni    ), // Asynchronous reset active low
    .test_i      ( test      ), // Testmode enable
    // slave ports [1:0]
    .slv         ( slv       ),
    .irq_o       ( irq       ), // interrupt output for each port
    .base_addr_i ( base_addr )  // base address for each port
  );
endmodule

module synth_axi_isolate #(
  parameter int unsigned NumPending   = 32'd16, // number of pending requests
  parameter int unsigned AxiIdWidth   = 32'd0,  // AXI ID width
  parameter int unsigned AxiAddrWidth = 32'd0,  // AXI address width
  parameter int unsigned AxiDataWidth = 32'd0,  // AXI data width
  parameter int unsigned AxiUserWidth = 32'd0   // AXI user width
) (
  input clk_i,
  input rst_ni
);

  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiIdWidth   ),
    .AXI_DATA_WIDTH ( AxiAddrWidth ),
    .AXI_ID_WIDTH   ( AxiDataWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth )
  ) axi[1:0] ();

  logic isolate, isolated;

  axi_isolate_intf #(
    .NUM_PENDING    ( NumPending   ), // number of pending requests
    .AXI_ID_WIDTH   ( AxiIdWidth   ), // AXI ID width
    .AXI_ADDR_WIDTH ( AxiAddrWidth ), // AXI address width
    .AXI_DATA_WIDTH ( AxiDataWidth ), // AXI data width
    .AXI_USER_WIDTH ( AxiUserWidth )  // AXI user width
  ) i_axi_isolate_dut (
    .clk_i,
    .rst_ni,
    .slv        ( axi[0]   ), // slave port
    .mst        ( axi[1]   ), // master port
    .isolate_i  ( isolate  ), // isolate master port from slave port
    .isolated_o ( isolated )  // master port is isolated from slave port
  );
endmodule

module synth_axi_modify_address #(
  parameter int unsigned AXI_SLV_PORT_ADDR_WIDTH = 0,
  parameter int unsigned AXI_MST_PORT_ADDR_WIDTH = 0,
  parameter int unsigned AXI_DATA_WIDTH = 0,
  parameter int unsigned AXI_ID_WIDTH = 0,
  parameter int unsigned AXI_USER_WIDTH = 0
) ();

  AXI_BUS #(
    .AXI_ADDR_WIDTH (AXI_SLV_PORT_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH)
  ) upstream ();

  AXI_BUS #(
    .AXI_ADDR_WIDTH (AXI_MST_PORT_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_USER_WIDTH (AXI_USER_WIDTH)
  ) downstream ();

  logic [AXI_MST_PORT_ADDR_WIDTH-1:0] mst_aw_addr,
                                      mst_ar_addr;
  axi_modify_address_intf #(
    .AXI_SLV_PORT_ADDR_WIDTH  (AXI_SLV_PORT_ADDR_WIDTH),
    .AXI_MST_PORT_ADDR_WIDTH  (AXI_MST_PORT_ADDR_WIDTH),
    .AXI_DATA_WIDTH           (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH             (AXI_ID_WIDTH),
    .AXI_USER_WIDTH           (AXI_USER_WIDTH)
  ) dut (
    .slv            (upstream),
    .mst_aw_addr_i  (mst_aw_addr),
    .mst_ar_addr_i  (mst_ar_addr),
    .mst            (downstream)
  );
endmodule

module synth_axi_serializer #(
  parameter int unsigned NumPending   = 32'd16, // number of pending requests
  parameter int unsigned AxiIdWidth   = 32'd0,  // AXI ID width
  parameter int unsigned AxiAddrWidth = 32'd0,  // AXI address width
  parameter int unsigned AxiDataWidth = 32'd0,  // AXI data width
  parameter int unsigned AxiUserWidth = 32'd0   // AXI user width
) (
  input clk_i,
  input rst_ni
);

  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiIdWidth   ),
    .AXI_DATA_WIDTH ( AxiAddrWidth ),
    .AXI_ID_WIDTH   ( AxiDataWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth )
  ) axi[1:0] ();

  axi_serializer_intf #(
    .MAX_READ_TXNS  ( NumPending   ), // Number of pending requests
    .MAX_WRITE_TXNS ( NumPending   ), // Number of pending requests
    .AXI_ID_WIDTH   ( AxiIdWidth   ), // AXI ID width
    .AXI_ADDR_WIDTH ( AxiAddrWidth ), // AXI address width
    .AXI_DATA_WIDTH ( AxiDataWidth ), // AXI data width
    .AXI_USER_WIDTH ( AxiUserWidth )  // AXI user width
  ) i_axi_isolate_dut (
    .clk_i,
    .rst_ni,
    .slv        ( axi[0]   ), // slave port
    .mst        ( axi[1]   )  // master port
  );
endmodule

module synth_axi_lite_regs #(
  parameter int unsigned REG_NUM_BYTES  = 32'd0,
  parameter int unsigned AXI_ADDR_WIDTH = 32'd0,
  parameter int unsigned AXI_DATA_WIDTH = 32'd0
) (
  input logic clk_i,
  input logic rst_ni
);
  typedef logic [7:0] byte_t;

  AXI_LITE #(
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH )
  ) slv ();

  logic  [REG_NUM_BYTES-1:0] wr_active, rd_active;
  byte_t [REG_NUM_BYTES-1:0] reg_d,     reg_q;
  logic  [REG_NUM_BYTES-1:0] reg_load;

  axi_lite_regs_intf #(
    .REG_NUM_BYTES  ( REG_NUM_BYTES          ),
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH         ),
    .AXI_DATA_WIDTH ( AXI_DATA_WIDTH         ),
    .PRIV_PROT_ONLY ( 1'd0                   ),
    .SECU_PROT_ONLY ( 1'd0                   ),
    .AXI_READ_ONLY  ( {REG_NUM_BYTES{1'b0}}  ),
    .REG_RST_VAL    ( {REG_NUM_BYTES{8'h00}} )
  ) i_axi_lite_regs (
    .clk_i,
    .rst_ni,
    .slv         ( slv         ),
    .wr_active_o ( wr_active   ),
    .rd_active_o ( rd_active   ),
    .reg_d_i     ( reg_d       ),
    .reg_load_i  ( reg_load    ),
    .reg_q_o     ( reg_q       )
  );
endmodule

module synth_axi_iw_converter # (
  parameter int unsigned AxiSlvPortIdWidth = 32'd0,
  parameter int unsigned AxiMstPortIdWidth = 32'd0,
  parameter int unsigned AxiSlvPortMaxUniqIds = 32'd0,
  parameter int unsigned AxiSlvPortMaxTxnsPerId = 32'd0,
  parameter int unsigned AxiSlvPortMaxTxns = 32'd0,
  parameter int unsigned AxiMstPortMaxUniqIds = 32'd0,
  parameter int unsigned AxiMstPortMaxTxnsPerId = 32'd0,
  parameter int unsigned AxiAddrWidth = 32'd0,
  parameter int unsigned AxiDataWidth = 32'd0,
  parameter int unsigned AxiUserWidth = 32'd0
) (
  input logic clk_i,
  input logic rst_ni
);
  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth      ),
    .AXI_DATA_WIDTH ( AxiDataWidth      ),
    .AXI_ID_WIDTH   ( AxiSlvPortIdWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth      )
  ) upstream ();
  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth      ),
    .AXI_DATA_WIDTH ( AxiDataWidth      ),
    .AXI_ID_WIDTH   ( AxiMstPortIdWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth      )
  ) downstream ();

  axi_iw_converter_intf #(
    .AXI_SLV_PORT_ID_WIDTH        (AxiSlvPortIdWidth      ),
    .AXI_MST_PORT_ID_WIDTH        (AxiMstPortIdWidth      ),
    .AXI_SLV_PORT_MAX_UNIQ_IDS    (AxiMstPortIdWidth      ),
    .AXI_SLV_PORT_MAX_TXNS_PER_ID (AxiSlvPortMaxTxnsPerId ),
    .AXI_SLV_PORT_MAX_TXNS        (AxiSlvPortMaxTxns      ),
    .AXI_MST_PORT_MAX_UNIQ_IDS    (AxiMstPortMaxUniqIds   ),
    .AXI_MST_PORT_MAX_TXNS_PER_ID (AxiMstPortMaxTxnsPerId ),
    .AXI_ADDR_WIDTH               (AxiAddrWidth           ),
    .AXI_DATA_WIDTH               (AxiDataWidth           ),
    .AXI_USER_WIDTH               (AxiUserWidth           )
  ) i_axi_iw_converter_dut (
    .clk_i,
    .rst_ni,
    .slv     ( upstream   ),
    .mst     ( downstream )
  );
endmodule


module synth_axi_to_mem_banked #(
  parameter int unsigned AxiDataWidth  = 32'd0,
  parameter int unsigned BankNum       = 32'd0,
  parameter int unsigned BankAddrWidth = 32'd0
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned AxiIdWidth    = 32'd10;
  localparam int unsigned AxiAddrWidth  = 32'd64;
  localparam int unsigned AxiStrbWidth  = AxiDataWidth / 32'd8;
  localparam int unsigned AxiUserWidth  = 32'd8;
  localparam int unsigned BankDataWidth = 32'd2 * AxiDataWidth / BankNum;
  localparam int unsigned BankStrbWidth = BankDataWidth / 32'd8;
  localparam int unsigned BankLatency   = 32'd1;

  typedef logic [BankAddrWidth-1:0] mem_addr_t;
  typedef logic [BankDataWidth-1:0] mem_data_t;
  typedef logic [BankStrbWidth-1:0] mem_strb_t;

  AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiIdWidth   ),
    .AXI_DATA_WIDTH ( AxiAddrWidth ),
    .AXI_ID_WIDTH   ( AxiDataWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth )
  ) axi ();

  // Misc signals
  logic                             test;
  logic           [1:0]             axi_to_mem_busy;
  // Signals for mem macros
  logic           [BankNum-1:0] mem_req;
  logic           [BankNum-1:0] mem_gnt;
  mem_addr_t      [BankNum-1:0] mem_addr;
  logic           [BankNum-1:0] mem_we;
  mem_data_t      [BankNum-1:0] mem_wdata;
  mem_strb_t      [BankNum-1:0] mem_be;
  axi_pkg::atop_t [BankNum-1:0] mem_atop;
  mem_data_t      [BankNum-1:0] mem_rdata;


  axi_to_mem_banked_intf #(
    .AXI_ID_WIDTH    ( AxiIdWidth    ),
    .AXI_ADDR_WIDTH  ( AxiAddrWidth  ),
    .AXI_DATA_WIDTH  ( AxiDataWidth  ),
    .AXI_USER_WIDTH  ( AxiUserWidth  ),
    .MEM_NUM_BANKS   ( BankNum       ),
    .MEM_ADDR_WIDTH  ( BankAddrWidth ),
    .MEM_DATA_WIDTH  ( BankDataWidth ),
    .MEM_LATENCY     ( BankLatency   )
  ) i_axi_to_mem_banked_intf (
    .clk_i,
    .rst_ni,
    .test_i            ( test            ),
    .slv               ( axi             ),
    .mem_req_o         ( mem_req         ),
    .mem_gnt_i         ( mem_gnt         ),
    .mem_add_o         ( mem_addr        ),
    .mem_we_o          ( mem_we          ),
    .mem_wdata_o       ( mem_wdata       ),
    .mem_be_o          ( mem_be          ),
    .mem_atop_o        ( mem_atop        ),
    .mem_rdata_i       ( mem_rdata       ),
    .axi_to_mem_busy_o ( axi_to_mem_busy )
  );

endmodule


module synth_axi_lite_dw_converter #(
  parameter int unsigned AXI_SLV_PORT_DATA_WIDTH = 32'd0,
  parameter int unsigned AXI_MST_PORT_DATA_WIDTH = 32'd0
) (
  input logic clk_i,
  input logic rst_ni
);

  localparam int unsigned AXI_ADDR_WIDTH = 32'd64;

  AXI_LITE #(
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH          ),
    .AXI_DATA_WIDTH ( AXI_SLV_PORT_DATA_WIDTH )
  ) slv_intf ();

  AXI_LITE #(
    .AXI_ADDR_WIDTH ( AXI_ADDR_WIDTH          ),
    .AXI_DATA_WIDTH ( AXI_MST_PORT_DATA_WIDTH )
  ) mst_intf ();

  axi_lite_dw_converter_intf #(
    .AXI_ADDR_WIDTH          ( AXI_ADDR_WIDTH          ),
    .AXI_SLV_PORT_DATA_WIDTH ( AXI_SLV_PORT_DATA_WIDTH ),
    .AXI_MST_PORT_DATA_WIDTH ( AXI_MST_PORT_DATA_WIDTH )
  ) i_axi_lite_dw_converter_intf (
    .clk_i,
    .rst_ni,
    .slv    ( slv_intf ),
    .mst    ( mst_intf )
  );

endmodule

module synth_axi_burst_splitter #(
  parameter int unsigned MaxReadTxns  = 32'd1,
  parameter int unsigned MaxWriteTxns = 32'd1,
  parameter bit unsigned FullBW       = 0,
  parameter int unsigned AddrWidth    = 32'd64,
  parameter int unsigned DataWidth    = 32'd64,
  parameter int unsigned IdWidth      = 32'd8,
  parameter int unsigned UserWidth    = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);

  `AXI_TYPEDEF_ALL(axi,
      logic [AddrWidth-1:0],
      logic [IdWidth-1:0],
      logic [DataWidth-1:0],
      logic [DataWidth/8-1:0],
      logic [UserWidth-1:0])

  axi_req_t  slv_req;
  axi_resp_t slv_resp;
  axi_req_t  mst_req;
  axi_resp_t mst_resp;

  axi_burst_splitter #(
    .MaxReadTxns  ( MaxReadTxns  ),
    .MaxWriteTxns ( MaxWriteTxns ),
    .FullBW       ( FullBW       ),
    .AddrWidth    ( AddrWidth    ),
    .DataWidth    ( DataWidth    ),
    .IdWidth      ( IdWidth      ),
    .UserWidth    ( UserWidth    ),
    .axi_req_t    ( axi_req_t    ),
    .axi_resp_t   ( axi_resp_t   )
  ) i_axi_burst_splitter (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req  ),
    .slv_resp_o ( slv_resp ),
    .mst_req_o  ( mst_req  ),
    .mst_resp_i ( mst_resp )
  );

endmodule


module synth_axi_burst_splitter_gran #(
  parameter int unsigned MaxReadTxns  = 32'd1,
  parameter int unsigned MaxWriteTxns = 32'd1,
  parameter bit unsigned FullBW       = 0,
  parameter int unsigned AddrWidth    = 32'd64,
  parameter int unsigned DataWidth    = 32'd64,
  parameter int unsigned IdWidth      = 32'd8,
  parameter int unsigned UserWidth    = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);

  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req;
  axi_resp_t slv_resp;
  axi_req_t  mst_req;
  axi_resp_t mst_resp;
  axi_pkg::len_t len_limit;

  axi_burst_splitter_gran #(
    .MaxReadTxns   ( MaxReadTxns   ),
    .MaxWriteTxns  ( MaxWriteTxns  ),
    .FullBW        ( FullBW        ),
    .CutPath       ( 1'b1          ), // differentiate from axi_burst_splitter
    .DisableChecks ( 1'b0          ),
    .AddrWidth     ( AddrWidth     ),
    .DataWidth     ( DataWidth     ),
    .IdWidth       ( IdWidth       ),
    .UserWidth     ( UserWidth     ),
    .axi_req_t     ( axi_req_t     ),
    .axi_resp_t    ( axi_resp_t    ),
    .axi_aw_chan_t ( axi_aw_chan_t ),
    .axi_w_chan_t  ( axi_w_chan_t  ),
    .axi_b_chan_t  ( axi_b_chan_t  ),
    .axi_ar_chan_t ( axi_ar_chan_t ),
    .axi_r_chan_t  ( axi_r_chan_t  )
  ) i_axi_burst_splitter_gran (
    .clk_i,
    .rst_ni,
    .len_limit_i ( len_limit ),
    .slv_req_i   ( slv_req   ),
    .slv_resp_o  ( slv_resp  ),
    .mst_req_o   ( mst_req   ),
    .mst_resp_i  ( mst_resp  )
  );

endmodule

module synth_axi_bus_compare #(
  parameter int unsigned FifoDepth = 32'd0,
  parameter bit          UseSize   = 1'b0,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);

  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  typedef logic [2**IdWidth-1:0] id_t;

  axi_req_t  axi_a_req_i,
             axi_a_req_o,
             axi_b_req_i,
             axi_b_req_o;
  axi_resp_t axi_a_rsp_o,
             axi_a_rsp_i,
             axi_b_rsp_o,
             axi_b_rsp_i;
  id_t       aw_mismatch_o,
             b_mismatch_o,
             ar_mismatch_o,
             r_mismatch_o;
  logic      testmode_i,
             w_mismatch_o,
             mismatch_o,
             busy_o;

  axi_bus_compare #(
    .AxiIdWidth    ( IdWidth       ),
    .FifoDepth     ( FifoDepth     ),
    .UseSize       ( UseSize       ), // differentiate from axi_burst_splitter
    .DataWidth     ( DataWidth     ),
    .axi_aw_chan_t ( axi_aw_chan_t ),
    .axi_w_chan_t  ( axi_w_chan_t  ),
    .axi_b_chan_t  ( axi_b_chan_t  ),
    .axi_ar_chan_t ( axi_ar_chan_t ),
    .axi_r_chan_t  ( axi_r_chan_t  ),
    .axi_req_t     ( axi_req_t     ),
    .axi_rsp_t     ( axi_resp_t    )
  ) i_axi_burst_splitter_gran (
    .*
  );

endmodule


module synth_axi_xbar #(
  parameter int unsigned NoSlvPorts = 32'd1,
  parameter int unsigned NoMstPorts = 32'd1,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd64,
  parameter int unsigned SlvIdWidth = 32'd4,
  parameter int unsigned UserWidth  = 32'd4
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned MstIdWidth       = SlvIdWidth + (NoSlvPorts > 1 ? $clog2(NoSlvPorts) : 0);
  localparam int unsigned MstPortsIdxWidth = (NoMstPorts > 1) ? $clog2(NoMstPorts) : 1;
  localparam axi_pkg::xbar_cfg_t XbarCfg = '{
    NoSlvPorts:         NoSlvPorts,
    NoMstPorts:         NoMstPorts,
    MaxMstTrans:        32'd4,
    MaxSlvTrans:        32'd4,
    FallThrough:        1'b0,
    LatencyMode:        axi_pkg::CUT_ALL_PORTS,
    PipelineStages:     32'd0,
    AxiIdWidthSlvPorts: SlvIdWidth,
    AxiIdUsedSlvPorts:  SlvIdWidth,
    UniqueIds:          1'b0,
    AxiAddrWidth:       AddrWidth,
    AxiDataWidth:       DataWidth,
    NoAddrRules:        NoMstPorts
  };

  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AddrWidth-1:0],
    logic [SlvIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AddrWidth-1:0],
    logic [MstIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  logic                                         test_i;
  slv_axi_req_t  [NoSlvPorts-1:0]              slv_ports_req_i;
  slv_axi_resp_t [NoSlvPorts-1:0]              slv_ports_resp_o;
  mst_axi_req_t  [NoMstPorts-1:0]              mst_ports_req_o;
  mst_axi_resp_t [NoMstPorts-1:0]              mst_ports_resp_i;
  axi_pkg::xbar_rule_64_t [NoMstPorts-1:0]    addr_map_i;
  logic           [NoSlvPorts-1:0]             en_default_mst_port_i;
  logic [NoSlvPorts-1:0][MstPortsIdxWidth-1:0] default_mst_port_i;

  axi_xbar #(
    .Cfg           ( XbarCfg                  ),
    .slv_aw_chan_t ( slv_axi_aw_chan_t        ),
    .mst_aw_chan_t ( mst_axi_aw_chan_t        ),
    .w_chan_t      ( slv_axi_w_chan_t         ),
    .slv_b_chan_t  ( slv_axi_b_chan_t         ),
    .mst_b_chan_t  ( mst_axi_b_chan_t         ),
    .slv_ar_chan_t ( slv_axi_ar_chan_t        ),
    .mst_ar_chan_t ( mst_axi_ar_chan_t        ),
    .slv_r_chan_t  ( slv_axi_r_chan_t         ),
    .mst_r_chan_t  ( mst_axi_r_chan_t         ),
    .slv_req_t     ( slv_axi_req_t           ),
    .slv_resp_t    ( slv_axi_resp_t          ),
    .mst_req_t     ( mst_axi_req_t           ),
    .mst_resp_t    ( mst_axi_resp_t          ),
    .rule_t        ( axi_pkg::xbar_rule_64_t )
  ) i_axi_xbar (
    .clk_i,
    .rst_ni,
    .test_i                ( test_i                ),
    .slv_ports_req_i       ( slv_ports_req_i       ),
    .slv_ports_resp_o      ( slv_ports_resp_o      ),
    .mst_ports_req_o       ( mst_ports_req_o       ),
    .mst_ports_resp_i      ( mst_ports_resp_i      ),
    .addr_map_i            ( addr_map_i            ),
    .en_default_mst_port_i ( en_default_mst_port_i ),
    .default_mst_port_i    ( default_mst_port_i    )
  );
endmodule


module synth_axi_cut #(
  parameter bit          Bypass    = 1'b0,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i, mst_req_o;
  axi_resp_t slv_resp_o, mst_resp_i;

  axi_cut #(
    .Bypass     ( Bypass        ),
    .aw_chan_t  ( axi_aw_chan_t ),
    .w_chan_t   ( axi_w_chan_t  ),
    .b_chan_t   ( axi_b_chan_t  ),
    .ar_chan_t  ( axi_ar_chan_t ),
    .r_chan_t   ( axi_r_chan_t  ),
    .axi_req_t  ( axi_req_t    ),
    .axi_resp_t ( axi_resp_t   )
  ) i_axi_cut (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_multicut #(
  parameter int unsigned NoCuts    = 32'd1,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i, mst_req_o;
  axi_resp_t slv_resp_o, mst_resp_i;

  axi_multicut #(
    .NoCuts     ( NoCuts        ),
    .aw_chan_t  ( axi_aw_chan_t ),
    .w_chan_t   ( axi_w_chan_t  ),
    .b_chan_t   ( axi_b_chan_t  ),
    .ar_chan_t  ( axi_ar_chan_t ),
    .r_chan_t   ( axi_r_chan_t  ),
    .axi_req_t  ( axi_req_t    ),
    .axi_resp_t ( axi_resp_t   )
  ) i_axi_multicut (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_fifo #(
  parameter int unsigned Depth     = 32'd1,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i, mst_req_o;
  axi_resp_t slv_resp_o, mst_resp_i;
  logic      test_i;

  axi_fifo #(
    .Depth      ( Depth         ),
    .aw_chan_t  ( axi_aw_chan_t ),
    .w_chan_t   ( axi_w_chan_t  ),
    .b_chan_t   ( axi_b_chan_t  ),
    .ar_chan_t  ( axi_ar_chan_t ),
    .r_chan_t   ( axi_r_chan_t  ),
    .axi_req_t  ( axi_req_t    ),
    .axi_resp_t ( axi_resp_t   )
  ) i_axi_fifo (
    .clk_i,
    .rst_ni,
    .test_i     ( test_i    ),
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_demux #(
  parameter int unsigned NoMstPorts = 32'd2,
  parameter int unsigned AxiIdWidth = 32'd8,
  parameter int unsigned MaxTrans   = 32'd8,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd64,
  parameter int unsigned UserWidth  = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned SelectWidth = (NoMstPorts > 1) ? $clog2(NoMstPorts) : 1;
  typedef logic [SelectWidth-1:0] select_t;

  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [AxiIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t               slv_req_i;
  axi_resp_t              slv_resp_o;
  select_t                slv_aw_select_i, slv_ar_select_i;
  axi_req_t  [NoMstPorts-1:0] mst_reqs_o;
  axi_resp_t [NoMstPorts-1:0] mst_resps_i;
  logic                   test_i;

  axi_demux #(
    .AxiIdWidth  ( AxiIdWidth   ),
    .aw_chan_t   ( axi_aw_chan_t ),
    .w_chan_t    ( axi_w_chan_t  ),
    .b_chan_t    ( axi_b_chan_t  ),
    .ar_chan_t   ( axi_ar_chan_t ),
    .r_chan_t    ( axi_r_chan_t  ),
    .axi_req_t   ( axi_req_t    ),
    .axi_resp_t  ( axi_resp_t   ),
    .NoMstPorts  ( NoMstPorts   ),
    .MaxTrans    ( MaxTrans      )
  ) i_axi_demux (
    .clk_i,
    .rst_ni,
    .test_i           ( test_i           ),
    .slv_req_i        ( slv_req_i        ),
    .slv_aw_select_i  ( slv_aw_select_i  ),
    .slv_ar_select_i  ( slv_ar_select_i  ),
    .slv_resp_o       ( slv_resp_o       ),
    .mst_reqs_o       ( mst_reqs_o       ),
    .mst_resps_i      ( mst_resps_i      )
  );
endmodule


module synth_axi_demux_simple #(
  parameter int unsigned NoMstPorts = 32'd2,
  parameter int unsigned AxiIdWidth = 32'd8,
  parameter int unsigned MaxTrans   = 32'd8,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd64,
  parameter int unsigned UserWidth  = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned SelectWidth = (NoMstPorts > 1) ? $clog2(NoMstPorts) : 1;
  typedef logic [SelectWidth-1:0] select_t;

  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [AxiIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t               slv_req_i;
  axi_resp_t              slv_resp_o;
  select_t                slv_aw_select_i, slv_ar_select_i;
  axi_req_t  [NoMstPorts-1:0] mst_reqs_o;
  axi_resp_t [NoMstPorts-1:0] mst_resps_i;
  logic                   test_i;

  axi_demux_simple #(
    .AxiIdWidth  ( AxiIdWidth  ),
    .axi_req_t   ( axi_req_t   ),
    .axi_resp_t  ( axi_resp_t  ),
    .NoMstPorts  ( NoMstPorts  ),
    .MaxTrans    ( MaxTrans     )
  ) i_axi_demux_simple (
    .clk_i,
    .rst_ni,
    .test_i           ( test_i           ),
    .slv_req_i        ( slv_req_i        ),
    .slv_aw_select_i  ( slv_aw_select_i  ),
    .slv_ar_select_i  ( slv_ar_select_i  ),
    .slv_resp_o       ( slv_resp_o       ),
    .mst_reqs_o       ( mst_reqs_o       ),
    .mst_resps_i      ( mst_resps_i      )
  );
endmodule


module synth_axi_mux #(
  parameter int unsigned NoSlvPorts = 32'd2,
  parameter int unsigned SlvIdWidth = 32'd4,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd64,
  parameter int unsigned UserWidth  = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned MstIdWidth = SlvIdWidth + (NoSlvPorts > 1 ? $clog2(NoSlvPorts) : 0);

  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AddrWidth-1:0],
    logic [SlvIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AddrWidth-1:0],
    logic [MstIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  slv_axi_req_t  [NoSlvPorts-1:0] slv_reqs_i;
  slv_axi_resp_t [NoSlvPorts-1:0] slv_resps_o;
  mst_axi_req_t                   mst_req_o;
  mst_axi_resp_t                  mst_resp_i;
  logic                           test_i;

  axi_mux #(
    .SlvAxiIDWidth ( SlvIdWidth          ),
    .slv_aw_chan_t ( slv_axi_aw_chan_t   ),
    .mst_aw_chan_t ( mst_axi_aw_chan_t   ),
    .w_chan_t      ( slv_axi_w_chan_t    ),
    .slv_b_chan_t  ( slv_axi_b_chan_t    ),
    .mst_b_chan_t  ( mst_axi_b_chan_t    ),
    .slv_ar_chan_t ( slv_axi_ar_chan_t   ),
    .mst_ar_chan_t ( mst_axi_ar_chan_t   ),
    .slv_r_chan_t  ( slv_axi_r_chan_t    ),
    .mst_r_chan_t  ( mst_axi_r_chan_t    ),
    .slv_req_t     ( slv_axi_req_t      ),
    .slv_resp_t    ( slv_axi_resp_t     ),
    .mst_req_t     ( mst_axi_req_t      ),
    .mst_resp_t    ( mst_axi_resp_t     ),
    .NoSlvPorts    ( NoSlvPorts         )
  ) i_axi_mux (
    .clk_i,
    .rst_ni,
    .test_i      ( test_i     ),
    .slv_reqs_i  ( slv_reqs_i ),
    .slv_resps_o ( slv_resps_o),
    .mst_req_o   ( mst_req_o  ),
    .mst_resp_i  ( mst_resp_i )
  );
endmodule


module synth_axi_dw_converter #(
  parameter int unsigned AxiSlvPortDataWidth = 32'd64,
  parameter int unsigned AxiMstPortDataWidth = 32'd32,
  parameter int unsigned AxiAddrWidth        = 32'd64,
  parameter int unsigned AxiIdWidth          = 32'd8,
  parameter int unsigned AxiUserWidth        = 32'd8,
  parameter int unsigned AxiMaxReads         = 32'd4
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AxiAddrWidth-1:0],
    logic [AxiIdWidth-1:0],
    logic [AxiSlvPortDataWidth-1:0],
    logic [AxiSlvPortDataWidth/8-1:0],
    logic [AxiUserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AxiAddrWidth-1:0],
    logic [AxiIdWidth-1:0],
    logic [AxiMstPortDataWidth-1:0],
    logic [AxiMstPortDataWidth/8-1:0],
    logic [AxiUserWidth-1:0])

  slv_axi_req_t  slv_req_i;
  slv_axi_resp_t slv_resp_o;
  mst_axi_req_t  mst_req_o;
  mst_axi_resp_t mst_resp_i;

  axi_dw_converter #(
    .AxiMaxReads         ( AxiMaxReads              ),
    .AxiSlvPortDataWidth ( AxiSlvPortDataWidth       ),
    .AxiMstPortDataWidth ( AxiMstPortDataWidth       ),
    .AxiAddrWidth        ( AxiAddrWidth              ),
    .AxiIdWidth          ( AxiIdWidth                ),
    .aw_chan_t           ( slv_axi_aw_chan_t         ),
    .mst_w_chan_t        ( mst_axi_w_chan_t          ),
    .slv_w_chan_t        ( slv_axi_w_chan_t          ),
    .b_chan_t            ( slv_axi_b_chan_t          ),
    .ar_chan_t           ( slv_axi_ar_chan_t         ),
    .mst_r_chan_t        ( mst_axi_r_chan_t          ),
    .slv_r_chan_t        ( slv_axi_r_chan_t          ),
    .axi_mst_req_t       ( mst_axi_req_t            ),
    .axi_mst_resp_t      ( mst_axi_resp_t           ),
    .axi_slv_req_t       ( slv_axi_req_t            ),
    .axi_slv_resp_t      ( slv_axi_resp_t           )
  ) i_axi_dw_converter (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_throttle #(
  parameter int unsigned MaxNumAwPending = 32'd4,
  parameter int unsigned MaxNumArPending = 32'd4,
  parameter int unsigned AddrWidth       = 32'd64,
  parameter int unsigned DataWidth       = 32'd64,
  parameter int unsigned IdWidth         = 32'd8,
  parameter int unsigned UserWidth       = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  localparam int unsigned WCntWidth = cf_math_pkg::idx_width(MaxNumAwPending);
  localparam int unsigned RCntWidth = cf_math_pkg::idx_width(MaxNumArPending);

  axi_req_t                  req_i, req_o;
  axi_resp_t                 rsp_o, rsp_i;
  logic [WCntWidth-1:0]      w_credit_i;
  logic [RCntWidth-1:0]      r_credit_i;

  axi_throttle #(
    .MaxNumAwPending ( MaxNumAwPending ),
    .MaxNumArPending ( MaxNumArPending ),
    .axi_req_t       ( axi_req_t      ),
    .axi_rsp_t       ( axi_resp_t     )
  ) i_axi_throttle (
    .clk_i,
    .rst_ni,
    .req_i      ( req_i      ),
    .rsp_o      ( rsp_o      ),
    .req_o      ( req_o      ),
    .rsp_i      ( rsp_i      ),
    .w_credit_i ( w_credit_i ),
    .r_credit_i ( r_credit_i )
  );
endmodule


module synth_axi_inval_filter #(
  parameter int unsigned MaxTxns     = 32'd4,
  parameter int unsigned AddrWidth   = 32'd64,
  parameter int unsigned L1LineWidth = 32'd64,
  parameter int unsigned DataWidth   = 32'd64,
  parameter int unsigned IdWidth     = 32'd8,
  parameter int unsigned UserWidth   = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t             slv_req_i, mst_req_o;
  axi_resp_t            slv_resp_o, mst_resp_i;
  logic                 en_i;
  logic [AddrWidth-1:0] inval_addr_o;
  logic                 inval_valid_o, inval_ready_i;

  axi_inval_filter #(
    .MaxTxns     ( MaxTxns         ),
    .AddrWidth   ( AddrWidth       ),
    .L1LineWidth ( L1LineWidth     ),
    .aw_chan_t   ( axi_aw_chan_t   ),
    .req_t       ( axi_req_t      ),
    .resp_t      ( axi_resp_t     )
  ) i_axi_inval_filter (
    .clk_i,
    .rst_ni,
    .en_i          ( en_i          ),
    .slv_req_i     ( slv_req_i     ),
    .slv_resp_o    ( slv_resp_o    ),
    .mst_req_o     ( mst_req_o     ),
    .mst_resp_i    ( mst_resp_i    ),
    .inval_addr_o  ( inval_addr_o  ),
    .inval_valid_o ( inval_valid_o ),
    .inval_ready_i ( inval_ready_i )
  );
endmodule


module synth_axi_id_serialize #(
  parameter int unsigned AxiSlvPortIdWidth    = 32'd8,
  parameter int unsigned AxiMstPortIdWidth    = 32'd4,
  parameter int unsigned AxiMstPortMaxUniqIds = 32'd8,
  parameter int unsigned AddrWidth            = 32'd64,
  parameter int unsigned DataWidth            = 32'd64,
  parameter int unsigned UserWidth            = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AddrWidth-1:0],
    logic [AxiSlvPortIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AddrWidth-1:0],
    logic [AxiMstPortIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  slv_axi_req_t  slv_req_i;
  slv_axi_resp_t slv_resp_o;
  mst_axi_req_t  mst_req_o;
  mst_axi_resp_t mst_resp_i;

  axi_id_serialize #(
    .AxiSlvPortIdWidth    ( AxiSlvPortIdWidth    ),
    .AxiSlvPortMaxTxns    ( 32'd16               ),
    .AxiMstPortIdWidth    ( AxiMstPortIdWidth    ),
    .AxiMstPortMaxUniqIds ( AxiMstPortMaxUniqIds ),
    .AxiMstPortMaxTxnsPerId( 32'd4              ),
    .AxiAddrWidth         ( AddrWidth            ),
    .AxiDataWidth         ( DataWidth            ),
    .AxiUserWidth         ( UserWidth            ),
    .slv_req_t            ( slv_axi_req_t        ),
    .slv_resp_t           ( slv_axi_resp_t       ),
    .mst_req_t            ( mst_axi_req_t        ),
    .mst_resp_t           ( mst_axi_resp_t       )
  ) i_axi_id_serialize (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_id_remap #(
  parameter int unsigned AxiSlvPortIdWidth    = 32'd8,
  parameter int unsigned AxiMstPortIdWidth    = 32'd4,
  parameter int unsigned AxiSlvPortMaxUniqIds = 32'd8,
  parameter int unsigned AxiMaxTxnsPerId      = 32'd4,
  parameter int unsigned AddrWidth            = 32'd64,
  parameter int unsigned DataWidth            = 32'd64,
  parameter int unsigned UserWidth            = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AddrWidth-1:0],
    logic [AxiSlvPortIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AddrWidth-1:0],
    logic [AxiMstPortIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  slv_axi_req_t  slv_req_i;
  slv_axi_resp_t slv_resp_o;
  mst_axi_req_t  mst_req_o;
  mst_axi_resp_t mst_resp_i;

  axi_id_remap #(
    .AxiSlvPortIdWidth    ( AxiSlvPortIdWidth    ),
    .AxiSlvPortMaxUniqIds ( AxiSlvPortMaxUniqIds ),
    .AxiMaxTxnsPerId      ( AxiMaxTxnsPerId      ),
    .AxiMstPortIdWidth    ( AxiMstPortIdWidth    ),
    .slv_req_t            ( slv_axi_req_t        ),
    .slv_resp_t           ( slv_axi_resp_t       ),
    .mst_req_t            ( mst_axi_req_t        ),
    .mst_resp_t           ( mst_axi_resp_t       )
  ) i_axi_id_remap (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_id_prepend #(
  parameter int unsigned NoBus             = 32'd1,
  parameter int unsigned AddrWidth         = 32'd64,
  parameter int unsigned DataWidth         = 32'd64,
  parameter int unsigned AxiIdWidthSlvPort = 32'd4,
  parameter int unsigned AxiIdWidthMstPort = 32'd6,
  parameter int unsigned UserWidth         = 32'd8
) ();
  localparam int unsigned PreIdWidth = AxiIdWidthMstPort - AxiIdWidthSlvPort;

  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AddrWidth-1:0],
    logic [AxiIdWidthSlvPort-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AddrWidth-1:0],
    logic [AxiIdWidthMstPort-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  logic             [PreIdWidth-1:0]  pre_id_i;
  slv_axi_aw_chan_t [NoBus-1:0]       slv_aw_chans_i;
  logic             [NoBus-1:0]       slv_aw_valids_i, slv_aw_readies_o;
  slv_axi_w_chan_t  [NoBus-1:0]       slv_w_chans_i;
  logic             [NoBus-1:0]       slv_w_valids_i,  slv_w_readies_o;
  slv_axi_b_chan_t  [NoBus-1:0]       slv_b_chans_o;
  logic             [NoBus-1:0]       slv_b_valids_o,  slv_b_readies_i;
  slv_axi_ar_chan_t [NoBus-1:0]       slv_ar_chans_i;
  logic             [NoBus-1:0]       slv_ar_valids_i, slv_ar_readies_o;
  slv_axi_r_chan_t  [NoBus-1:0]       slv_r_chans_o;
  logic             [NoBus-1:0]       slv_r_valids_o,  slv_r_readies_i;
  mst_axi_aw_chan_t [NoBus-1:0]       mst_aw_chans_o;
  logic             [NoBus-1:0]       mst_aw_valids_o, mst_aw_readies_i;
  mst_axi_w_chan_t  [NoBus-1:0]       mst_w_chans_o;
  logic             [NoBus-1:0]       mst_w_valids_o,  mst_w_readies_i;
  mst_axi_b_chan_t  [NoBus-1:0]       mst_b_chans_i;
  logic             [NoBus-1:0]       mst_b_valids_i,  mst_b_readies_o;
  mst_axi_ar_chan_t [NoBus-1:0]       mst_ar_chans_o;
  logic             [NoBus-1:0]       mst_ar_valids_o, mst_ar_readies_i;
  mst_axi_r_chan_t  [NoBus-1:0]       mst_r_chans_i;
  logic             [NoBus-1:0]       mst_r_valids_i,  mst_r_readies_o;

  axi_id_prepend #(
    .NoBus             ( NoBus              ),
    .AxiIdWidthSlvPort ( AxiIdWidthSlvPort  ),
    .AxiIdWidthMstPort ( AxiIdWidthMstPort  ),
    .slv_aw_chan_t     ( slv_axi_aw_chan_t  ),
    .slv_w_chan_t      ( slv_axi_w_chan_t   ),
    .slv_b_chan_t      ( slv_axi_b_chan_t   ),
    .slv_ar_chan_t     ( slv_axi_ar_chan_t  ),
    .slv_r_chan_t      ( slv_axi_r_chan_t   ),
    .mst_aw_chan_t     ( mst_axi_aw_chan_t  ),
    .mst_w_chan_t      ( mst_axi_w_chan_t   ),
    .mst_b_chan_t      ( mst_axi_b_chan_t   ),
    .mst_ar_chan_t     ( mst_axi_ar_chan_t  ),
    .mst_r_chan_t      ( mst_axi_r_chan_t   )
  ) i_axi_id_prepend (
    .pre_id_i,
    .slv_aw_chans_i,  .slv_aw_valids_i,  .slv_aw_readies_o,
    .slv_w_chans_i,   .slv_w_valids_i,   .slv_w_readies_o,
    .slv_b_chans_o,   .slv_b_valids_o,   .slv_b_readies_i,
    .slv_ar_chans_i,  .slv_ar_valids_i,  .slv_ar_readies_o,
    .slv_r_chans_o,   .slv_r_valids_o,   .slv_r_readies_i,
    .mst_aw_chans_o,  .mst_aw_valids_o,  .mst_aw_readies_i,
    .mst_w_chans_o,   .mst_w_valids_o,   .mst_w_readies_i,
    .mst_b_chans_i,   .mst_b_valids_i,   .mst_b_readies_o,
    .mst_ar_chans_o,  .mst_ar_valids_o,  .mst_ar_readies_i,
    .mst_r_chans_i,   .mst_r_valids_i,   .mst_r_readies_o
  );
endmodule


module synth_axi_demux_id_counters #(
  parameter int unsigned AxiIdBits    = 32'd3,
  parameter int unsigned CounterWidth = 32'd4,
  parameter int unsigned NoMstPorts   = 32'd4
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned SelectWidth = $clog2(NoMstPorts);
  typedef logic [SelectWidth-1:0] mst_port_select_t;

  logic [AxiIdBits-1:0]   lookup_axi_id_i;
  mst_port_select_t        lookup_mst_select_o;
  logic                    lookup_mst_select_occupied_o;
  logic                    full_o;
  logic [AxiIdBits-1:0]   push_axi_id_i;
  mst_port_select_t        push_mst_select_i;
  logic                    push_i;
  logic [AxiIdBits-1:0]   inject_axi_id_i;
  logic                    inject_i;
  logic [AxiIdBits-1:0]   pop_axi_id_i;
  logic                    pop_i;
  logic                    any_outstanding_trx_o;

  axi_demux_id_counters #(
    .AxiIdBits         ( AxiIdBits          ),
    .CounterWidth      ( CounterWidth       ),
    .mst_port_select_t ( mst_port_select_t  )
  ) i_axi_demux_id_counters (
    .clk_i,
    .rst_ni,
    .lookup_axi_id_i              ( lookup_axi_id_i              ),
    .lookup_mst_select_o          ( lookup_mst_select_o          ),
    .lookup_mst_select_occupied_o ( lookup_mst_select_occupied_o ),
    .full_o                       ( full_o                       ),
    .push_axi_id_i                ( push_axi_id_i                ),
    .push_mst_select_i            ( push_mst_select_i            ),
    .push_i                       ( push_i                       ),
    .inject_axi_id_i              ( inject_axi_id_i              ),
    .inject_i                     ( inject_i                     ),
    .pop_axi_id_i                 ( pop_axi_id_i                 ),
    .pop_i                        ( pop_i                        ),
    .any_outstanding_trx_o        ( any_outstanding_trx_o        )
  );
endmodule


module synth_axi_to_mem #(
  parameter int unsigned NumBanks  = 32'd1,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  typedef logic [AddrWidth-1:0]          addr_t;
  typedef logic [DataWidth/NumBanks-1:0] mem_data_t;
  typedef logic [DataWidth/NumBanks/8-1:0] mem_strb_t;

  axi_req_t                        axi_req_i;
  axi_resp_t                       axi_resp_o;
  logic                            busy_o;
  logic           [NumBanks-1:0]   mem_req_o;
  logic           [NumBanks-1:0]   mem_gnt_i;
  addr_t          [NumBanks-1:0]   mem_addr_o;
  mem_data_t      [NumBanks-1:0]   mem_wdata_o;
  mem_strb_t      [NumBanks-1:0]   mem_strb_o;
  axi_pkg::atop_t [NumBanks-1:0]   mem_atop_o;
  logic           [NumBanks-1:0]   mem_we_o;
  logic           [NumBanks-1:0]   mem_rvalid_i;
  mem_data_t      [NumBanks-1:0]   mem_rdata_i;

  axi_to_mem #(
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t ),
    .AddrWidth  ( AddrWidth  ),
    .DataWidth  ( DataWidth  ),
    .IdWidth    ( IdWidth    ),
    .NumBanks   ( NumBanks   )
  ) i_axi_to_mem (
    .clk_i,
    .rst_ni,
    .busy_o       ( busy_o      ),
    .axi_req_i    ( axi_req_i   ),
    .axi_resp_o   ( axi_resp_o  ),
    .mem_req_o    ( mem_req_o   ),
    .mem_gnt_i    ( mem_gnt_i   ),
    .mem_addr_o   ( mem_addr_o  ),
    .mem_wdata_o  ( mem_wdata_o ),
    .mem_strb_o   ( mem_strb_o  ),
    .mem_atop_o   ( mem_atop_o  ),
    .mem_we_o     ( mem_we_o    ),
    .mem_rvalid_i ( mem_rvalid_i),
    .mem_rdata_i  ( mem_rdata_i )
  );
endmodule


module synth_axi_to_mem_interleaved #(
  parameter int unsigned NumBanks  = 32'd1,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  typedef logic [AddrWidth-1:0]            addr_t;
  typedef logic [DataWidth/NumBanks-1:0]   mem_data_t;
  typedef logic [DataWidth/NumBanks/8-1:0] mem_strb_t;

  axi_req_t                        axi_req_i;
  axi_resp_t                       axi_resp_o;
  logic                            test_i, busy_o;
  logic           [NumBanks-1:0]   mem_req_o;
  logic           [NumBanks-1:0]   mem_gnt_i;
  addr_t          [NumBanks-1:0]   mem_addr_o;
  mem_data_t      [NumBanks-1:0]   mem_wdata_o;
  mem_strb_t      [NumBanks-1:0]   mem_strb_o;
  axi_pkg::atop_t [NumBanks-1:0]   mem_atop_o;
  logic           [NumBanks-1:0]   mem_we_o;
  logic           [NumBanks-1:0]   mem_rvalid_i;
  mem_data_t      [NumBanks-1:0]   mem_rdata_i;

  axi_to_mem_interleaved #(
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t ),
    .AddrWidth  ( AddrWidth  ),
    .DataWidth  ( DataWidth  ),
    .IdWidth    ( IdWidth    ),
    .NumBanks   ( NumBanks   )
  ) i_axi_to_mem_interleaved (
    .clk_i,
    .rst_ni,
    .test_i       ( test_i      ),
    .busy_o       ( busy_o      ),
    .axi_req_i    ( axi_req_i   ),
    .axi_resp_o   ( axi_resp_o  ),
    .mem_req_o    ( mem_req_o   ),
    .mem_gnt_i    ( mem_gnt_i   ),
    .mem_addr_o   ( mem_addr_o  ),
    .mem_wdata_o  ( mem_wdata_o ),
    .mem_strb_o   ( mem_strb_o  ),
    .mem_atop_o   ( mem_atop_o  ),
    .mem_we_o     ( mem_we_o    ),
    .mem_rvalid_i ( mem_rvalid_i),
    .mem_rdata_i  ( mem_rdata_i )
  );
endmodule


module synth_axi_to_mem_split #(
  parameter int unsigned MemDataWidth = 32'd32,
  parameter int unsigned AddrWidth    = 32'd64,
  parameter int unsigned AxiDataWidth = 32'd64,
  parameter int unsigned IdWidth      = 32'd8,
  parameter int unsigned UserWidth    = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned NumMemPorts = 2 * AxiDataWidth / MemDataWidth;

  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [AxiDataWidth-1:0],
    logic [AxiDataWidth/8-1:0],
    logic [UserWidth-1:0])

  typedef logic [AddrWidth-1:0]      addr_t;
  typedef logic [MemDataWidth-1:0]   mem_data_t;
  typedef logic [MemDataWidth/8-1:0] mem_strb_t;

  axi_req_t                            axi_req_i;
  axi_resp_t                           axi_resp_o;
  logic                                test_i, busy_o;
  logic           [NumMemPorts-1:0]    mem_req_o;
  logic           [NumMemPorts-1:0]    mem_gnt_i;
  addr_t          [NumMemPorts-1:0]    mem_addr_o;
  mem_data_t      [NumMemPorts-1:0]    mem_wdata_o;
  mem_strb_t      [NumMemPorts-1:0]    mem_strb_o;
  axi_pkg::atop_t [NumMemPorts-1:0]    mem_atop_o;
  logic           [NumMemPorts-1:0]    mem_we_o;
  logic           [NumMemPorts-1:0]    mem_rvalid_i;
  mem_data_t      [NumMemPorts-1:0]    mem_rdata_i;

  axi_to_mem_split #(
    .axi_req_t    ( axi_req_t    ),
    .axi_resp_t   ( axi_resp_t   ),
    .AddrWidth    ( AddrWidth    ),
    .AxiDataWidth ( AxiDataWidth ),
    .IdWidth      ( IdWidth      ),
    .MemDataWidth ( MemDataWidth )
  ) i_axi_to_mem_split (
    .clk_i,
    .rst_ni,
    .test_i       ( test_i      ),
    .busy_o       ( busy_o      ),
    .axi_req_i    ( axi_req_i   ),
    .axi_resp_o   ( axi_resp_o  ),
    .mem_req_o    ( mem_req_o   ),
    .mem_gnt_i    ( mem_gnt_i   ),
    .mem_addr_o   ( mem_addr_o  ),
    .mem_wdata_o  ( mem_wdata_o ),
    .mem_strb_o   ( mem_strb_o  ),
    .mem_atop_o   ( mem_atop_o  ),
    .mem_we_o     ( mem_we_o    ),
    .mem_rvalid_i ( mem_rvalid_i),
    .mem_rdata_i  ( mem_rdata_i )
  );
endmodule


module synth_axi_to_detailed_mem #(
  parameter int unsigned NumBanks  = 32'd1,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  typedef logic [AddrWidth-1:0]            addr_t;
  typedef logic [DataWidth/NumBanks-1:0]   mem_data_t;
  typedef logic [DataWidth/NumBanks/8-1:0] mem_strb_t;
  typedef logic [IdWidth-1:0]              mem_id_t;
  typedef logic [UserWidth-1:0]            mem_user_t;

  axi_req_t                          axi_req_i;
  axi_resp_t                         axi_resp_o;
  logic                              busy_o;
  logic              [NumBanks-1:0]  mem_req_o;
  logic              [NumBanks-1:0]  mem_gnt_i;
  addr_t             [NumBanks-1:0]  mem_addr_o;
  mem_data_t         [NumBanks-1:0]  mem_wdata_o;
  mem_strb_t         [NumBanks-1:0]  mem_strb_o;
  axi_pkg::atop_t    [NumBanks-1:0]  mem_atop_o;
  logic              [NumBanks-1:0]  mem_lock_o;
  logic              [NumBanks-1:0]  mem_we_o;
  mem_id_t           [NumBanks-1:0]  mem_id_o;
  mem_user_t         [NumBanks-1:0]  mem_user_o;
  axi_pkg::cache_t   [NumBanks-1:0]  mem_cache_o;
  axi_pkg::prot_t    [NumBanks-1:0]  mem_prot_o;
  axi_pkg::qos_t     [NumBanks-1:0]  mem_qos_o;
  axi_pkg::region_t  [NumBanks-1:0]  mem_region_o;
  logic              [NumBanks-1:0]  mem_rvalid_i;
  mem_data_t         [NumBanks-1:0]  mem_rdata_i;
  logic              [NumBanks-1:0]  mem_err_i;
  logic              [NumBanks-1:0]  mem_exokay_i;

  axi_to_detailed_mem #(
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t ),
    .AddrWidth  ( AddrWidth  ),
    .DataWidth  ( DataWidth  ),
    .IdWidth    ( IdWidth    ),
    .UserWidth  ( UserWidth  ),
    .NumBanks   ( NumBanks   )
  ) i_axi_to_detailed_mem (
    .clk_i,
    .rst_ni,
    .busy_o       ( busy_o       ),
    .axi_req_i    ( axi_req_i    ),
    .axi_resp_o   ( axi_resp_o   ),
    .mem_req_o    ( mem_req_o    ),
    .mem_gnt_i    ( mem_gnt_i    ),
    .mem_addr_o   ( mem_addr_o   ),
    .mem_wdata_o  ( mem_wdata_o  ),
    .mem_strb_o   ( mem_strb_o   ),
    .mem_atop_o   ( mem_atop_o   ),
    .mem_lock_o   ( mem_lock_o   ),
    .mem_we_o     ( mem_we_o     ),
    .mem_id_o     ( mem_id_o     ),
    .mem_user_o   ( mem_user_o   ),
    .mem_cache_o  ( mem_cache_o  ),
    .mem_prot_o   ( mem_prot_o   ),
    .mem_qos_o    ( mem_qos_o    ),
    .mem_region_o ( mem_region_o ),
    .mem_rvalid_i ( mem_rvalid_i ),
    .mem_rdata_i  ( mem_rdata_i  ),
    .mem_err_i    ( mem_err_i    ),
    .mem_exokay_i ( mem_exokay_i )
  );
endmodule


module synth_axi_from_mem #(
  parameter int unsigned MaxRequests  = 32'd4,
  parameter int unsigned MemAddrWidth = 32'd64,
  parameter int unsigned AxiAddrWidth = 32'd64,
  parameter int unsigned DataWidth    = 32'd64,
  parameter int unsigned IdWidth      = 32'd8,
  parameter int unsigned UserWidth    = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AxiAddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  logic                     mem_req_i;
  logic [MemAddrWidth-1:0]  mem_addr_i;
  logic                     mem_we_i;
  logic [DataWidth-1:0]     mem_wdata_i;
  logic [DataWidth/8-1:0]   mem_be_i;
  logic                     mem_gnt_o;
  logic                     mem_rsp_valid_o;
  logic [DataWidth-1:0]     mem_rsp_rdata_o;
  logic                     mem_rsp_error_o;
  axi_pkg::cache_t          slv_aw_cache_i;
  axi_pkg::cache_t          slv_ar_cache_i;
  axi_req_t                 axi_req_o;
  axi_resp_t                axi_rsp_i;

  axi_from_mem #(
    .MemAddrWidth ( MemAddrWidth ),
    .AxiAddrWidth ( AxiAddrWidth ),
    .DataWidth    ( DataWidth    ),
    .MaxRequests  ( MaxRequests  ),
    .axi_req_t    ( axi_req_t   ),
    .axi_rsp_t    ( axi_resp_t  )
  ) i_axi_from_mem (
    .clk_i,
    .rst_ni,
    .mem_req_i       ( mem_req_i       ),
    .mem_addr_i      ( mem_addr_i      ),
    .mem_we_i        ( mem_we_i        ),
    .mem_wdata_i     ( mem_wdata_i     ),
    .mem_be_i        ( mem_be_i        ),
    .mem_gnt_o       ( mem_gnt_o       ),
    .mem_rsp_valid_o ( mem_rsp_valid_o ),
    .mem_rsp_rdata_o ( mem_rsp_rdata_o ),
    .mem_rsp_error_o ( mem_rsp_error_o ),
    .slv_aw_cache_i  ( slv_aw_cache_i  ),
    .slv_ar_cache_i  ( slv_ar_cache_i  ),
    .axi_req_o       ( axi_req_o       ),
    .axi_rsp_i       ( axi_rsp_i       )
  );
endmodule


module synth_axi_lite_from_mem #(
  parameter int unsigned MaxRequests  = 32'd4,
  parameter int unsigned MemAddrWidth = 32'd64,
  parameter int unsigned AxiAddrWidth = 32'd64,
  parameter int unsigned DataWidth    = 32'd32
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_LITE_TYPEDEF_ALL(lite,
    logic [AxiAddrWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0])

  logic                    mem_req_i;
  logic [MemAddrWidth-1:0] mem_addr_i;
  logic                    mem_we_i;
  logic [DataWidth-1:0]    mem_wdata_i;
  logic [DataWidth/8-1:0]  mem_be_i;
  logic                    mem_gnt_o;
  logic                    mem_rsp_valid_o;
  logic [DataWidth-1:0]    mem_rsp_rdata_o;
  logic                    mem_rsp_error_o;
  lite_req_t               axi_req_o;
  lite_resp_t              axi_rsp_i;

  axi_lite_from_mem #(
    .MemAddrWidth ( MemAddrWidth ),
    .AxiAddrWidth ( AxiAddrWidth ),
    .DataWidth    ( DataWidth    ),
    .MaxRequests  ( MaxRequests  ),
    .axi_req_t    ( lite_req_t  ),
    .axi_rsp_t    ( lite_resp_t )
  ) i_axi_lite_from_mem (
    .clk_i,
    .rst_ni,
    .mem_req_i       ( mem_req_i       ),
    .mem_addr_i      ( mem_addr_i      ),
    .mem_we_i        ( mem_we_i        ),
    .mem_wdata_i     ( mem_wdata_i     ),
    .mem_be_i        ( mem_be_i        ),
    .mem_gnt_o       ( mem_gnt_o       ),
    .mem_rsp_valid_o ( mem_rsp_valid_o ),
    .mem_rsp_rdata_o ( mem_rsp_rdata_o ),
    .mem_rsp_error_o ( mem_rsp_error_o ),
    .axi_req_o       ( axi_req_o       ),
    .axi_rsp_i       ( axi_rsp_i       )
  );
endmodule


module synth_axi_interleaved_xbar #(
  parameter int unsigned NoSlvPorts = 32'd1,
  parameter int unsigned NoMstPorts = 32'd1,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd64,
  parameter int unsigned SlvIdWidth = 32'd4,
  parameter int unsigned UserWidth  = 32'd4
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned MstIdWidth       = SlvIdWidth + (NoSlvPorts > 1 ? $clog2(NoSlvPorts) : 0);
  localparam int unsigned MstPortsIdxWidth = (NoMstPorts > 1) ? $clog2(NoMstPorts) : 1;
  localparam axi_pkg::xbar_cfg_t XbarCfg = '{
    NoSlvPorts:         NoSlvPorts,
    NoMstPorts:         NoMstPorts,
    MaxMstTrans:        32'd4,
    MaxSlvTrans:        32'd4,
    FallThrough:        1'b0,
    LatencyMode:        axi_pkg::CUT_ALL_PORTS,
    PipelineStages:     32'd0,
    AxiIdWidthSlvPorts: SlvIdWidth,
    AxiIdUsedSlvPorts:  SlvIdWidth,
    UniqueIds:          1'b0,
    AxiAddrWidth:       AddrWidth,
    AxiDataWidth:       DataWidth,
    NoAddrRules:        NoMstPorts
  };

  `AXI_TYPEDEF_ALL(slv_axi,
    logic [AddrWidth-1:0],
    logic [SlvIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  `AXI_TYPEDEF_ALL(mst_axi,
    logic [AddrWidth-1:0],
    logic [MstIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  logic                                         test_i;
  slv_axi_req_t  [NoSlvPorts-1:0]              slv_ports_req_i;
  slv_axi_resp_t [NoSlvPorts-1:0]              slv_ports_resp_o;
  mst_axi_req_t  [NoMstPorts-1:0]              mst_ports_req_o;
  mst_axi_resp_t [NoMstPorts-1:0]              mst_ports_resp_i;
  axi_pkg::xbar_rule_64_t [NoMstPorts-1:0]    addr_map_i;
  logic                                        interleaved_mode_ena_i;
  logic           [NoSlvPorts-1:0]             en_default_mst_port_i;
  logic [NoSlvPorts-1:0][MstPortsIdxWidth-1:0] default_mst_port_i;

  axi_interleaved_xbar #(
    .Cfg           ( XbarCfg                  ),
    .slv_aw_chan_t ( slv_axi_aw_chan_t        ),
    .mst_aw_chan_t ( mst_axi_aw_chan_t        ),
    .w_chan_t      ( slv_axi_w_chan_t         ),
    .slv_b_chan_t  ( slv_axi_b_chan_t         ),
    .mst_b_chan_t  ( mst_axi_b_chan_t         ),
    .slv_ar_chan_t ( slv_axi_ar_chan_t        ),
    .mst_ar_chan_t ( mst_axi_ar_chan_t        ),
    .slv_r_chan_t  ( slv_axi_r_chan_t         ),
    .mst_r_chan_t  ( mst_axi_r_chan_t         ),
    .slv_req_t     ( slv_axi_req_t           ),
    .slv_resp_t    ( slv_axi_resp_t          ),
    .mst_req_t     ( mst_axi_req_t           ),
    .mst_resp_t    ( mst_axi_resp_t          ),
    .rule_t        ( axi_pkg::xbar_rule_64_t )
  ) i_axi_interleaved_xbar (
    .clk_i,
    .rst_ni,
    .test_i                  ( test_i                  ),
    .slv_ports_req_i         ( slv_ports_req_i         ),
    .slv_ports_resp_o        ( slv_ports_resp_o        ),
    .mst_ports_req_o         ( mst_ports_req_o         ),
    .mst_ports_resp_i        ( mst_ports_resp_i        ),
    .addr_map_i              ( addr_map_i              ),
    .interleaved_mode_ena_i  ( interleaved_mode_ena_i  ),
    .en_default_mst_port_i   ( en_default_mst_port_i   ),
    .default_mst_port_i      ( default_mst_port_i      )
  );
endmodule


module synth_axi_xp #(
  parameter int unsigned NumSlvPorts = 32'd1,
  parameter int unsigned NumMstPorts = 32'd1,
  parameter int unsigned AddrWidth   = 32'd64,
  parameter int unsigned DataWidth   = 32'd64,
  parameter int unsigned IdWidth     = 32'd4,
  parameter int unsigned UserWidth   = 32'd4,
  parameter int unsigned NumAddrRules = NumMstPorts
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  localparam axi_pkg::xbar_cfg_t XbarCfg = '{
    NoSlvPorts:         NumSlvPorts,
    NoMstPorts:         NumMstPorts,
    MaxMstTrans:        32'd4,
    MaxSlvTrans:        32'd4,
    FallThrough:        1'b0,
    LatencyMode:        axi_pkg::CUT_ALL_PORTS,
    PipelineStages:     32'd0,
    AxiIdWidthSlvPorts: IdWidth,
    AxiIdUsedSlvPorts:  IdWidth,
    UniqueIds:          1'b0,
    AxiAddrWidth:       AddrWidth,
    AxiDataWidth:       DataWidth,
    NoAddrRules:        NumAddrRules
  };

  logic                             test_en_i;
  axi_req_t  [NumSlvPorts-1:0]      slv_req_i;
  axi_resp_t [NumSlvPorts-1:0]      slv_resp_o;
  axi_req_t  [NumMstPorts-1:0]      mst_req_o;
  axi_resp_t [NumMstPorts-1:0]      mst_resp_i;
  axi_pkg::xbar_rule_64_t [NumAddrRules-1:0] addr_map_i;

  axi_xp #(
    .ATOPs                  ( 1'b1                    ),
    .Cfg                    ( XbarCfg                 ),
    .NumSlvPorts            ( NumSlvPorts             ),
    .NumMstPorts            ( NumMstPorts             ),
    .Connectivity           ( '1                      ),
    .AxiAddrWidth           ( AddrWidth               ),
    .AxiDataWidth           ( DataWidth               ),
    .AxiIdWidth             ( IdWidth                 ),
    .AxiUserWidth           ( UserWidth               ),
    .AxiSlvPortMaxUniqIds   ( 32'd4                   ),
    .AxiSlvPortMaxTxnsPerId ( 32'd4                   ),
    .AxiSlvPortMaxTxns      ( 32'd16                  ),
    .AxiMstPortMaxUniqIds   ( 32'd4                   ),
    .AxiMstPortMaxTxnsPerId ( 32'd4                   ),
    .NumAddrRules           ( NumAddrRules            ),
    .axi_req_t              ( axi_req_t               ),
    .axi_resp_t             ( axi_resp_t              ),
    .rule_t                 ( axi_pkg::xbar_rule_64_t )
  ) i_axi_xp (
    .clk_i,
    .rst_ni,
    .test_en_i  ( test_en_i  ),
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i ),
    .addr_map_i ( addr_map_i )
  );
endmodule


module synth_axi_burst_unwrap #(
  parameter int unsigned MaxReadTxns  = 32'd4,
  parameter int unsigned MaxWriteTxns = 32'd4,
  parameter int unsigned AddrWidth    = 32'd64,
  parameter int unsigned DataWidth    = 32'd64,
  parameter int unsigned IdWidth      = 32'd8,
  parameter int unsigned UserWidth    = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i, mst_req_o;
  axi_resp_t slv_resp_o, mst_resp_i;

  axi_burst_unwrap #(
    .MaxReadTxns  ( MaxReadTxns  ),
    .MaxWriteTxns ( MaxWriteTxns ),
    .AddrWidth    ( AddrWidth    ),
    .DataWidth    ( DataWidth    ),
    .IdWidth      ( IdWidth      ),
    .UserWidth    ( UserWidth    ),
    .axi_req_t    ( axi_req_t   ),
    .axi_resp_t   ( axi_resp_t  )
  ) i_axi_burst_unwrap (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_delayer #(
  parameter int unsigned AddrWidth       = 32'd64,
  parameter int unsigned DataWidth       = 32'd64,
  parameter int unsigned IdWidth         = 32'd8,
  parameter int unsigned UserWidth       = 32'd8,
  parameter int unsigned FixedDelayInput  = 32'd1,
  parameter int unsigned FixedDelayOutput = 32'd1
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i, mst_req_o;
  axi_resp_t slv_resp_o, mst_resp_i;

  axi_delayer #(
    .aw_chan_t          ( axi_aw_chan_t    ),
    .w_chan_t           ( axi_w_chan_t     ),
    .b_chan_t           ( axi_b_chan_t     ),
    .ar_chan_t          ( axi_ar_chan_t    ),
    .r_chan_t           ( axi_r_chan_t     ),
    .axi_req_t          ( axi_req_t       ),
    .axi_resp_t         ( axi_resp_t      ),
    .FixedDelayInput    ( FixedDelayInput  ),
    .FixedDelayOutput   ( FixedDelayOutput )
  ) i_axi_delayer (
    .clk_i,
    .rst_ni,
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_rw_split #(
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i;
  axi_resp_t slv_resp_o;
  axi_req_t  mst_read_req_o,  mst_write_req_o;
  axi_resp_t mst_read_resp_i, mst_write_resp_i;

  axi_rw_split #(
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t )
  ) i_axi_rw_split (
    .clk_i,
    .rst_ni,
    .slv_req_i       ( slv_req_i       ),
    .slv_resp_o      ( slv_resp_o      ),
    .mst_read_req_o  ( mst_read_req_o  ),
    .mst_read_resp_i ( mst_read_resp_i ),
    .mst_write_req_o ( mst_write_req_o ),
    .mst_write_resp_i( mst_write_resp_i)
  );
endmodule


module synth_axi_zero_mem #(
  parameter int unsigned NumBanks  = 32'd1,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  axi_req_i;
  axi_resp_t axi_resp_o;
  logic      busy_o;

  axi_zero_mem #(
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t ),
    .AddrWidth  ( AddrWidth  ),
    .DataWidth  ( DataWidth  ),
    .IdWidth    ( IdWidth    ),
    .NumBanks   ( NumBanks   )
  ) i_axi_zero_mem (
    .clk_i,
    .rst_ni,
    .busy_o     ( busy_o     ),
    .axi_req_i  ( axi_req_i  ),
    .axi_resp_o ( axi_resp_o )
  );
endmodule


module synth_axi_lfsr #(
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  logic      testmode_i;
  axi_req_t  req_i;
  axi_resp_t rsp_o;
  logic      w_ser_data_i, w_ser_data_o, w_ser_en_i;
  logic      r_ser_data_i, r_ser_data_o, r_ser_en_i;

  axi_lfsr #(
    .DataWidth ( DataWidth  ),
    .AddrWidth ( AddrWidth  ),
    .IdWidth   ( IdWidth    ),
    .UserWidth ( UserWidth  ),
    .axi_req_t ( axi_req_t  ),
    .axi_rsp_t ( axi_resp_t )
  ) i_axi_lfsr (
    .clk_i,
    .rst_ni,
    .testmode_i    ( testmode_i    ),
    .req_i         ( req_i         ),
    .rsp_o         ( rsp_o         ),
    .w_ser_data_i  ( w_ser_data_i  ),
    .w_ser_data_o  ( w_ser_data_o  ),
    .w_ser_en_i    ( w_ser_en_i    ),
    .r_ser_data_i  ( r_ser_data_i  ),
    .r_ser_data_o  ( r_ser_data_o  ),
    .r_ser_en_i    ( r_ser_en_i    )
  );
endmodule


module synth_axi_lite_lfsr #(
  parameter int unsigned DataWidth = 32'd32,
  parameter int unsigned AddrWidth = 32'd64
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_LITE_TYPEDEF_ALL(lite,
    logic [AddrWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0])

  logic       testmode_i;
  lite_req_t  req_i;
  lite_resp_t rsp_o;
  logic       w_ser_data_i, w_ser_data_o, w_ser_en_i;
  logic       r_ser_data_i, r_ser_data_o, r_ser_en_i;

  axi_lite_lfsr #(
    .DataWidth       ( DataWidth    ),
    .axi_lite_req_t  ( lite_req_t  ),
    .axi_lite_rsp_t  ( lite_resp_t )
  ) i_axi_lite_lfsr (
    .clk_i,
    .rst_ni,
    .testmode_i    ( testmode_i    ),
    .req_i         ( req_i         ),
    .rsp_o         ( rsp_o         ),
    .w_ser_data_i  ( w_ser_data_i  ),
    .w_ser_data_o  ( w_ser_data_o  ),
    .w_ser_en_i    ( w_ser_en_i    ),
    .r_ser_data_i  ( r_ser_data_i  ),
    .r_ser_data_o  ( r_ser_data_o  ),
    .r_ser_en_i    ( r_ser_en_i    )
  );
endmodule


module synth_axi_slave_compare #(
  parameter int unsigned FifoDepth = 32'd4,
  parameter bit          UseSize   = 1'b0,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])
  typedef logic [2**IdWidth-1:0] id_t;

  axi_req_t axi_mst_req_i,
            axi_ref_req_o,
            axi_test_req_o;
  axi_resp_t axi_mst_rsp_o,
             axi_ref_rsp_i,
             axi_test_rsp_i;
  id_t       aw_mismatch_o,
             b_mismatch_o,
             ar_mismatch_o,
             r_mismatch_o;
  logic      testmode_i,
             w_mismatch_o,
             mismatch_o,
             busy_o;

  axi_slave_compare #(
    .AxiIdWidth    ( IdWidth       ),
    .FifoDepth     ( FifoDepth     ),
    .UseSize       ( UseSize       ),
    .DataWidth     ( DataWidth     ),
    .axi_aw_chan_t ( axi_aw_chan_t ),
    .axi_w_chan_t  ( axi_w_chan_t  ),
    .axi_b_chan_t  ( axi_b_chan_t  ),
    .axi_ar_chan_t ( axi_ar_chan_t ),
    .axi_r_chan_t  ( axi_r_chan_t  ),
    .axi_req_t     ( axi_req_t    ),
    .axi_rsp_t     ( axi_resp_t   )
  ) i_axi_slave_compare (
    .*
  );
endmodule


module synth_axi_fifo_delay_dyn #(
  parameter int unsigned MaxDelay  = 32'd64,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned DelayWidth = $clog2(MaxDelay) + 1;

  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t                slv_req_i, mst_req_o;
  axi_resp_t               slv_resp_o, mst_resp_i;
  logic [DelayWidth-1:0]   aw_delay_i, w_delay_i, b_delay_i, ar_delay_i, r_delay_i;

  axi_fifo_delay_dyn #(
    .aw_chan_t  ( axi_aw_chan_t ),
    .w_chan_t   ( axi_w_chan_t  ),
    .b_chan_t   ( axi_b_chan_t  ),
    .ar_chan_t  ( axi_ar_chan_t ),
    .r_chan_t   ( axi_r_chan_t  ),
    .axi_req_t  ( axi_req_t    ),
    .axi_resp_t ( axi_resp_t   ),
    .MaxDelay   ( MaxDelay     )
  ) i_axi_fifo_delay_dyn (
    .clk_i,
    .rst_ni,
    .aw_delay_i ( aw_delay_i ),
    .w_delay_i  ( w_delay_i  ),
    .b_delay_i  ( b_delay_i  ),
    .ar_delay_i ( ar_delay_i ),
    .r_delay_i  ( r_delay_i  ),
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o ),
    .mst_req_o  ( mst_req_o  ),
    .mst_resp_i ( mst_resp_i )
  );
endmodule


module synth_axi_err_slv #(
  parameter int unsigned AxiIdWidth = 32'd8,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd64,
  parameter int unsigned UserWidth  = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [AxiIdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_req_i;
  axi_resp_t slv_resp_o;
  logic      test_i;

  axi_err_slv #(
    .AxiIdWidth ( AxiIdWidth ),
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t )
  ) i_axi_err_slv (
    .clk_i,
    .rst_ni,
    .test_i     ( test_i     ),
    .slv_req_i  ( slv_req_i  ),
    .slv_resp_o ( slv_resp_o )
  );
endmodule


module synth_axi_cdc_src #(
  parameter int unsigned LogDepth  = 32'd2,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  src_req_i;
  axi_resp_t src_resp_o;
  axi_aw_chan_t [2**LogDepth-1:0] async_data_master_aw_data_o;
  logic         [LogDepth:0]      async_data_master_aw_wptr_o;
  logic         [LogDepth:0]      async_data_master_aw_rptr_i;
  axi_w_chan_t  [2**LogDepth-1:0] async_data_master_w_data_o;
  logic         [LogDepth:0]      async_data_master_w_wptr_o;
  logic         [LogDepth:0]      async_data_master_w_rptr_i;
  axi_b_chan_t  [2**LogDepth-1:0] async_data_master_b_data_i;
  logic         [LogDepth:0]      async_data_master_b_wptr_i;
  logic         [LogDepth:0]      async_data_master_b_rptr_o;
  axi_ar_chan_t [2**LogDepth-1:0] async_data_master_ar_data_o;
  logic         [LogDepth:0]      async_data_master_ar_wptr_o;
  logic         [LogDepth:0]      async_data_master_ar_rptr_i;
  axi_r_chan_t  [2**LogDepth-1:0] async_data_master_r_data_i;
  logic         [LogDepth:0]      async_data_master_r_wptr_i;
  logic         [LogDepth:0]      async_data_master_r_rptr_o;

  axi_cdc_src #(
    .LogDepth   ( LogDepth      ),
    .aw_chan_t  ( axi_aw_chan_t ),
    .w_chan_t   ( axi_w_chan_t  ),
    .b_chan_t   ( axi_b_chan_t  ),
    .ar_chan_t  ( axi_ar_chan_t ),
    .r_chan_t   ( axi_r_chan_t  ),
    .axi_req_t  ( axi_req_t    ),
    .axi_resp_t ( axi_resp_t   )
  ) i_axi_cdc_src (
    .src_clk_i  ( clk_i   ),
    .src_rst_ni ( rst_ni  ),
    .src_req_i  ( src_req_i  ),
    .src_resp_o ( src_resp_o ),
    .async_data_master_aw_data_o ( async_data_master_aw_data_o ),
    .async_data_master_aw_wptr_o ( async_data_master_aw_wptr_o ),
    .async_data_master_aw_rptr_i ( async_data_master_aw_rptr_i ),
    .async_data_master_w_data_o  ( async_data_master_w_data_o  ),
    .async_data_master_w_wptr_o  ( async_data_master_w_wptr_o  ),
    .async_data_master_w_rptr_i  ( async_data_master_w_rptr_i  ),
    .async_data_master_b_data_i  ( async_data_master_b_data_i  ),
    .async_data_master_b_wptr_i  ( async_data_master_b_wptr_i  ),
    .async_data_master_b_rptr_o  ( async_data_master_b_rptr_o  ),
    .async_data_master_ar_data_o ( async_data_master_ar_data_o ),
    .async_data_master_ar_wptr_o ( async_data_master_ar_wptr_o ),
    .async_data_master_ar_rptr_i ( async_data_master_ar_rptr_i ),
    .async_data_master_r_data_i  ( async_data_master_r_data_i  ),
    .async_data_master_r_wptr_i  ( async_data_master_r_wptr_i  ),
    .async_data_master_r_rptr_o  ( async_data_master_r_rptr_o  )
  );
endmodule


module synth_axi_cdc_dst #(
  parameter int unsigned LogDepth  = 32'd2,
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_aw_chan_t [2**LogDepth-1:0] async_data_slave_aw_data_i;
  logic         [LogDepth:0]      async_data_slave_aw_wptr_i;
  logic         [LogDepth:0]      async_data_slave_aw_rptr_o;
  axi_w_chan_t  [2**LogDepth-1:0] async_data_slave_w_data_i;
  logic         [LogDepth:0]      async_data_slave_w_wptr_i;
  logic         [LogDepth:0]      async_data_slave_w_rptr_o;
  axi_b_chan_t  [2**LogDepth-1:0] async_data_slave_b_data_o;
  logic         [LogDepth:0]      async_data_slave_b_wptr_o;
  logic         [LogDepth:0]      async_data_slave_b_rptr_i;
  axi_ar_chan_t [2**LogDepth-1:0] async_data_slave_ar_data_i;
  logic         [LogDepth:0]      async_data_slave_ar_wptr_i;
  logic         [LogDepth:0]      async_data_slave_ar_rptr_o;
  axi_r_chan_t  [2**LogDepth-1:0] async_data_slave_r_data_o;
  logic         [LogDepth:0]      async_data_slave_r_wptr_o;
  logic         [LogDepth:0]      async_data_slave_r_rptr_i;
  axi_req_t                       dst_req_o;
  axi_resp_t                      dst_resp_i;

  axi_cdc_dst #(
    .LogDepth   ( LogDepth      ),
    .aw_chan_t  ( axi_aw_chan_t ),
    .w_chan_t   ( axi_w_chan_t  ),
    .b_chan_t   ( axi_b_chan_t  ),
    .ar_chan_t  ( axi_ar_chan_t ),
    .r_chan_t   ( axi_r_chan_t  ),
    .axi_req_t  ( axi_req_t    ),
    .axi_resp_t ( axi_resp_t   )
  ) i_axi_cdc_dst (
    .async_data_slave_aw_data_i ( async_data_slave_aw_data_i ),
    .async_data_slave_aw_wptr_i ( async_data_slave_aw_wptr_i ),
    .async_data_slave_aw_rptr_o ( async_data_slave_aw_rptr_o ),
    .async_data_slave_w_data_i  ( async_data_slave_w_data_i  ),
    .async_data_slave_w_wptr_i  ( async_data_slave_w_wptr_i  ),
    .async_data_slave_w_rptr_o  ( async_data_slave_w_rptr_o  ),
    .async_data_slave_b_data_o  ( async_data_slave_b_data_o  ),
    .async_data_slave_b_wptr_o  ( async_data_slave_b_wptr_o  ),
    .async_data_slave_b_rptr_i  ( async_data_slave_b_rptr_i  ),
    .async_data_slave_ar_data_i ( async_data_slave_ar_data_i ),
    .async_data_slave_ar_wptr_i ( async_data_slave_ar_wptr_i ),
    .async_data_slave_ar_rptr_o ( async_data_slave_ar_rptr_o ),
    .async_data_slave_r_data_o  ( async_data_slave_r_data_o  ),
    .async_data_slave_r_wptr_o  ( async_data_slave_r_wptr_o  ),
    .async_data_slave_r_rptr_i  ( async_data_slave_r_rptr_i  ),
    .dst_clk_i  ( clk_i    ),
    .dst_rst_ni ( rst_ni   ),
    .dst_req_o  ( dst_req_o  ),
    .dst_resp_i ( dst_resp_i )
  );
endmodule


module synth_axi_lite_demux #(
  parameter int unsigned NoMstPorts = 32'd2,
  parameter int unsigned MaxTrans   = 32'd8,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd32
) (
  input logic clk_i,
  input logic rst_ni
);
  localparam int unsigned SelectWidth = (NoMstPorts > 1) ? $clog2(NoMstPorts) : 1;
  typedef logic [SelectWidth-1:0] select_t;

  `AXI_LITE_TYPEDEF_ALL(lite,
    logic [AddrWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0])

  lite_req_t               slv_req_i;
  lite_resp_t              slv_resp_o;
  select_t                 slv_aw_select_i, slv_ar_select_i;
  lite_req_t  [NoMstPorts-1:0] mst_reqs_o;
  lite_resp_t [NoMstPorts-1:0] mst_resps_i;
  logic                    test_i;

  axi_lite_demux #(
    .aw_chan_t   ( lite_aw_chan_t ),
    .w_chan_t    ( lite_w_chan_t  ),
    .b_chan_t    ( lite_b_chan_t  ),
    .ar_chan_t   ( lite_ar_chan_t ),
    .r_chan_t    ( lite_r_chan_t  ),
    .axi_req_t  ( lite_req_t    ),
    .axi_resp_t ( lite_resp_t   ),
    .NoMstPorts  ( NoMstPorts   ),
    .MaxTrans    ( MaxTrans      )
  ) i_axi_lite_demux (
    .clk_i,
    .rst_ni,
    .test_i           ( test_i           ),
    .slv_req_i        ( slv_req_i        ),
    .slv_aw_select_i  ( slv_aw_select_i  ),
    .slv_ar_select_i  ( slv_ar_select_i  ),
    .slv_resp_o       ( slv_resp_o       ),
    .mst_reqs_o       ( mst_reqs_o       ),
    .mst_resps_i      ( mst_resps_i      )
  );
endmodule


module synth_axi_lite_mux #(
  parameter int unsigned NoSlvPorts = 32'd2,
  parameter int unsigned MaxTrans   = 32'd8,
  parameter int unsigned AddrWidth  = 32'd64,
  parameter int unsigned DataWidth  = 32'd32
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_LITE_TYPEDEF_ALL(lite,
    logic [AddrWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0])

  lite_req_t  [NoSlvPorts-1:0] slv_reqs_i;
  lite_resp_t [NoSlvPorts-1:0] slv_resps_o;
  lite_req_t                   mst_req_o;
  lite_resp_t                  mst_resp_i;
  logic                        test_i;

  axi_lite_mux #(
    .aw_chan_t   ( lite_aw_chan_t ),
    .w_chan_t    ( lite_w_chan_t  ),
    .b_chan_t    ( lite_b_chan_t  ),
    .ar_chan_t   ( lite_ar_chan_t ),
    .r_chan_t    ( lite_r_chan_t  ),
    .axi_req_t  ( lite_req_t    ),
    .axi_resp_t ( lite_resp_t   ),
    .NoSlvPorts  ( NoSlvPorts   ),
    .MaxTrans    ( MaxTrans      )
  ) i_axi_lite_mux (
    .clk_i,
    .rst_ni,
    .test_i      ( test_i      ),
    .slv_reqs_i  ( slv_reqs_i  ),
    .slv_resps_o ( slv_resps_o ),
    .mst_req_o   ( mst_req_o   ),
    .mst_resp_i  ( mst_resp_i  )
  );
endmodule

module synth_axi_rw_join #(
  parameter int unsigned AddrWidth = 32'd64,
  parameter int unsigned DataWidth = 32'd64,
  parameter int unsigned IdWidth   = 32'd8,
  parameter int unsigned UserWidth = 32'd8
) (
  input logic clk_i,
  input logic rst_ni
);
  `AXI_TYPEDEF_ALL(axi,
    logic [AddrWidth-1:0],
    logic [IdWidth-1:0],
    logic [DataWidth-1:0],
    logic [DataWidth/8-1:0],
    logic [UserWidth-1:0])

  axi_req_t  slv_read_req_i,  slv_write_req_i;
  axi_resp_t slv_read_resp_o, slv_write_resp_o;
  axi_req_t  mst_req_o;
  axi_resp_t mst_resp_i;

  axi_rw_join #(
    .axi_req_t  ( axi_req_t  ),
    .axi_resp_t ( axi_resp_t )
  ) i_axi_rw_join (
    .clk_i,
    .rst_ni,
    .slv_read_req_i   ( slv_read_req_i   ),
    .slv_read_resp_o  ( slv_read_resp_o  ),
    .slv_write_req_i  ( slv_write_req_i  ),
    .slv_write_resp_o ( slv_write_resp_o ),
    .mst_req_o        ( mst_req_o        ),
    .mst_resp_i       ( mst_resp_i       )
  );
endmodule
