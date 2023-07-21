using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using MathLib;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{
    public class VrpStopAction : VrpAction
    {
        public VrpStopAction(int vehicle_index, DateTime stoptime, Vector2f stoppos, int tol_time_arrival_secs, string stop_name, int service_time_secs)
            : base(vehicle_index)
        {

            _stoptime = stoptime;
            _stoppos = stoppos;
            _tol_time_arrival_secs = tol_time_arrival_secs;
            _service_time_secs = service_time_secs;
            _stop_name = stop_name;
        }

        #region Attribs
        protected DateTime _stoptime;
        protected Vector2f _stoppos;
        protected int _tol_time_arrival_secs;
        protected int _service_time_secs;
        protected string _stop_name;
        #endregion

        #region properties
        public Vector2f StopPosition
        {
            set
            {
                _stoppos = value;
            }

            get
            {
                return _stoppos;
            }
        }

        /// <summary>
        /// earliest stop time
        /// </summary>
        public DateTime EST
        {
            get
            {
                return _stoptime.AddSeconds(-_tol_time_arrival_secs);
            }
        }

        /// <summary>
        /// lastet stop time
        /// </summary>
        public DateTime LST
        {
            get
            {
                return _stoptime.AddSeconds(+_tol_time_arrival_secs);
            }
        }

        public TimeSpan ServiceTime
        {

            get
            {
                return new TimeSpan(0, 0, _service_time_secs);
            }
        }
		
		public DateTime StopTime
		{
			set {
				_stoptime = value;
			}
			
			get {
				return _stoptime;
			}
		}
		
		public string StopName
		{
			set {
				_stop_name = value;
			}
			
			get {
				return _stop_name;
			}
		}
        #endregion

        /// <summary>
        /// deep copy - clones the current action
        /// </summary>
        /// <returns></returns>
        public object Clone() {
            VrpStopAction clone = new VrpStopAction(_vehicle_index, _stoptime, _stoppos, _tol_time_arrival_secs, _stop_name, _service_time_secs);
            return clone;
        }

        public override string ToString()
        {
            return String.Format("V{0}ST", _vehicle_index);
        }

        public override bool Equals(object obj)
        {
            // If parameter is null return false.
            if (obj == null)
            {
                return false;
            }

            // If parameter cannot be cast to Point return false.
            VrpStopAction action = obj as VrpStopAction;
            if (action == null)
            {
                return false;
            }

            if (this.VehicleIndex != action.VehicleIndex)
                return false;
            if (this.StopPosition != action.StopPosition)
                return false;
            if (this.StopTime != action.StopTime)
                return false;

            return true;
        }

    }
}
