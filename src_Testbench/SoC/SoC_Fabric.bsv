// Copyright (c) 2013-2019 Bluespec, Inc. All Rights Reserved

package SoC_Fabric;

// ================================================================
// Defines a SoC Fabric that is a specialization of AXI4_Lite_Fabric
// for this particular SoC.

// ================================================================
// Project imports

import AXI4_Types  :: *;
import AXI4_Fabric :: *;

import Fabric_Defs :: *;    // for Wd_Addr, Wd_Data, Wd_User
import SoC_Map     :: *;    // for Num_Masters, Num_Slaves

// ================================================================
// Slave address decoder
// Identifies whether a given addr is legal and, if so, which  slave services it.

typedef Bit #(TLog #(Num_Slaves))  Slave_Num;

// ================================================================
// Specialization of parameterized AXI4 fabric for this SoC.

typedef AXI4_Fabric_IFC #(Num_Masters,
			  Num_Slaves,
			  Wd_Id,
			  Wd_Addr,
			  Wd_Data,
			  Wd_User)  Fabric_AXI4_IFC;

// ----------------

(* synthesize *)
module mkFabric_AXI4 (Fabric_AXI4_IFC);

   // Decode from the SAME declarative table the manifest is emitted
   // from (SoC_Map.fn_soc_memmap): fabric-decoded regions only, first
   // match wins.  Semantically identical to the former if-chain, but
   // the address ranges now have a single source of truth.
   List #(MemRegion) soc_memmap = fn_soc_memmap ();

   function Bool fn_is_fabric_region (MemRegion r);
      return r.fabric_decoded;
   endfunction

   function Tuple3 #(Fabric_Addr, Fabric_Addr, Slave_Num) fn_mk_entry (MemRegion r);
      return tuple3 (zeroExtend (r.base),
                     zeroExtend (r.base + r.size),
                     fn_dev_slave_num (r.device));
   endfunction

   List #(Tuple3 #(Fabric_Addr, Fabric_Addr, Slave_Num)) axi_decode_tbl =
      List::map (fn_mk_entry, List::filter (fn_is_fabric_region, soc_memmap));

   function Tuple2 #(Bool, Slave_Num) fn_decode (Fabric_Addr addr,
                                                 List #(Tuple3 #(Fabric_Addr,
                                                                 Fabric_Addr,
                                                                 Slave_Num)) rs);
      if (rs == nil)
         return tuple2 (False, ?);
      else begin
         match { .rb, .rl, .rsn } = List::head (rs);
         if ((rb <= addr) && (addr < rl))
            return tuple2 (True, rsn);
         else
            return fn_decode (addr, List::tail (rs));
      end
   endfunction

   function Tuple2 #(Bool, Slave_Num) fn_addr_to_slave_num  (Fabric_Addr addr);
      return fn_decode (addr, axi_decode_tbl);
   endfunction

   AXI4_Fabric_IFC #(Num_Masters, Num_Slaves, Wd_Id, Wd_Addr, Wd_Data, Wd_User)
       fabric <- mkAXI4_Fabric (fn_addr_to_slave_num);

   return fabric;
endmodule

// ================================================================
// Peripheral bridge: a 1-master sub-fabric on main-fabric slave port periph_slave_num, decoding
// the bridged devices' regions from the same SoC_Map table.  Both fabrics' xactors are FIFOs, so
// the hop between them is registered and the low-speed devices can sit anywhere on the die.
// The main fabric only forwards addresses inside bridged regions, so this decoder never sees an
// unmapped address (its err_events stay empty).

typedef AXI4_Fabric_IFC #(1,
			  Num_Periph,
			  Wd_Id,
			  Wd_Addr,
			  Wd_Data,
			  Wd_User)  Periph_Fabric_AXI4_IFC;

(* synthesize *)
module mkPeriph_Fabric_AXI4 (Periph_Fabric_AXI4_IFC);

   function Bool fn_is_periph_region (MemRegion r);
      return r.fabric_decoded && fn_dev_is_periph (r.device);
   endfunction

   function Tuple3 #(Fabric_Addr, Fabric_Addr, Periph_Num) fn_mk_entry (MemRegion r);
      return tuple3 (zeroExtend (r.base),
                     zeroExtend (r.base + r.size),
                     fn_dev_periph_num (r.device));
   endfunction

   List #(Tuple3 #(Fabric_Addr, Fabric_Addr, Periph_Num)) tbl =
      List::map (fn_mk_entry, List::filter (fn_is_periph_region, fn_soc_memmap ()));

   function Tuple2 #(Bool, Periph_Num) fn_decode (Fabric_Addr addr,
                                                  List #(Tuple3 #(Fabric_Addr,
                                                                  Fabric_Addr,
                                                                  Periph_Num)) rs);
      if (rs == nil)
         return tuple2 (False, ?);
      else begin
         match { .rb, .rl, .rpn } = List::head (rs);
         if ((rb <= addr) && (addr < rl))
            return tuple2 (True, rpn);
         else
            return fn_decode (addr, List::tail (rs));
      end
   endfunction

   function Tuple2 #(Bool, Periph_Num) fn_addr_to_periph_num (Fabric_Addr addr);
      return fn_decode (addr, tbl);
   endfunction

   Periph_Fabric_AXI4_IFC fabric <- mkAXI4_Fabric (fn_addr_to_periph_num);
   return fabric;
endmodule

// ================================================================

endpackage
