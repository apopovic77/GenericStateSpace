using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{
    public class VrpRequestAction : VrpAction
    {
        public VrpRequestAction(int vehicle_index, int request_index, bool is_pickup) :base(vehicle_index)
        {
            _request_index = request_index;
            _is_pickup = is_pickup;
        }


        public bool IsPickup
        {
            set
            {
                _is_pickup = value;
            }

            get
            {
                return _is_pickup;
            }
        }

        public int RequestIndex
        {
            set
            {
                _request_index = value;
            }

            get
            {
                return _request_index;
            }
        }

        public VrpRequestAction Clone()
        {
            VrpRequestAction clone = new VrpRequestAction(_vehicle_index, _request_index, _is_pickup);
            return clone;
        }

        public override string ToString()
        {
            return "V" + _vehicle_index + "R" + _request_index + ((_is_pickup) ? "P" : "D");
        }

        protected int _request_index = -1;
        protected bool _is_pickup;
    }
}
