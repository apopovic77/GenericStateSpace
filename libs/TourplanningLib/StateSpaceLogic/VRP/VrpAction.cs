using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using Action = Logicx.Optimization.GenericStateSpace.Action;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{
    public class VrpAction : Action
    {
        public VrpAction(int vehicle_index)
        {
            _vehicle_index = vehicle_index;
        }
		
		public int VehicleIndex
		{
			set {
				_vehicle_index = value;
			}
			
			get {
				return _vehicle_index;
			}
		}


        public override string ToString()
        {
            return "V" + _vehicle_index;
        }

        protected int _vehicle_index=-1;
    }
}
