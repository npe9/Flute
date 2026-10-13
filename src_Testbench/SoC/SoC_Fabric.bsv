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

endpackage
