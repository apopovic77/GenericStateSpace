using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using Logicx.Optimization.Tourplanning.NearestNeighbour;
using MathLib;
using Action = Logicx.Optimization.GenericStateSpace.Action;


namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{
    public class VrpState : State, ICloneable
    {
        #region construction
        public VrpState(int count_vehicles)
        {
            _time_at_curr_vehiclepos = new DateTime[count_vehicles];
            _curr_vehicle_pos = new Vector2f[count_vehicles];
            _vehicle_utilization = new int[count_vehicles];
            _last_served_request_index = new int[count_vehicles];
            _last_served_request_ispickup = new bool[count_vehicles];
            //_poss_time_shift_back = 0;
            //_poss_time_shift_forward = 0;
            _depth_state = 0;
            for (int i = 0; i < count_vehicles; i++)
            {
                _last_served_request_index[i] = -1;
                _last_served_request_ispickup[i] = true;
            }
        }
        public VrpState(int count_vehicles, VrpState prev_state, VrpAction action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_Actions,VrpStateSpace statespace )
            : base(prev_state, action,possible_Actions,statespace)
		{
            _curr_traveldistance_all = prev_state._curr_traveldistance_all;
            _curr_traveltime_all_secs = prev_state._curr_traveltime_all_secs;
            //_poss_time_shift_back = prev_state._poss_time_shift_back;
            //_poss_time_shift_forward = prev_state._poss_time_shift_forward;
            _vehicle_utilization = (int[])prev_state._vehicle_utilization.Clone();
            _last_served_request_index = (int[])prev_state._last_served_request_index.Clone();
            _last_served_request_ispickup = (bool[])prev_state._last_served_request_ispickup.Clone();
            _curr_vehicle_pos = (Vector2f[])prev_state._curr_vehicle_pos.Clone();
            _time_at_curr_vehiclepos = (DateTime[])prev_state._time_at_curr_vehiclepos.Clone();
            _depth_state = prev_state._depth_state + 1;

            if (action is VrpRequestAction)
            {
                _last_served_request_index[action.VehicleIndex] = ((VrpRequestAction)action).RequestIndex;
                _last_served_request_ispickup[action.VehicleIndex] = ((VrpRequestAction)action).IsPickup;
            }

		}

        public override float CurrentTargetValue {
            get {
                //// first poss, use the travel distance as a target value
                //return CurrTraveldistanceAll;

                ////second poss, use the travel time
                //VrpAction vrpaction = (VrpAction)_action;
                //TimeSpan ts = _time_at_curr_vehiclepos[vrpaction.VehicleIndex] - ((VrpState)_prev_state)._time_at_curr_vehiclepos[vrpaction.VehicleIndex];
                //return (float)ts.TotalSeconds;

                return CurrTraveltimeAllSecs;

            }
        }

        /// <summary>
        /// this time value indicates the global time
        /// where are we right now? Having this state arrived, the time should be CurrTime.
        /// </summary>
        public DateTime[] CurrTime
        {
            set
            {
                _time_at_curr_vehiclepos = value;
            }

            get
            {
                return _time_at_curr_vehiclepos;
            }
        }
		
        /// <summary>
        /// gibt die letzte Action des Vehicles zurück
        /// </summary>	
		public Vector2f[] CurrVehiclePos
		{
			set {
				_curr_vehicle_pos = value;
			}
			
			get {
				return _curr_vehicle_pos;
			}
		}
		
		public int[] VehicleUtilization
		{
			set {
				_vehicle_utilization = value;
			}
			
			get {
				return _vehicle_utilization;
			}
		}
		
		public float CurrTraveldistanceAll
		{
			set {
				_curr_traveldistance_all = value;
			}
			
			get {
				return _curr_traveldistance_all;
			}
		}


        public bool[] LastServedRequestIspickup
        {
            set
            {
                _last_served_request_ispickup = value;
            }

            get
            {
                return _last_served_request_ispickup;
            }
        }

        public int[] LastServedRequestIndex
        {
            set
            {
                _last_served_request_index = value;
            }

            get
            {
                return _last_served_request_index;
            }
        }
		
		public float CurrTraveltimeAllSecs
		{
			set {
				_curr_traveltime_all_secs = value;
			}
			
			get {
				return _curr_traveltime_all_secs;
			}
		}
        #endregion

        /// <summary>
        /// We dont use this approach to set the action dependencies because of performance lack
        /// instead of comparing each action with each action we explicitly set the action depedency 
        /// at action creation time 
        /// </summary>
        public override void SetActionDependencies()
        {   
            for (int i = 0; i < _possible_actions_index.Count; i++)
            {
                VrpAction action_i = (VrpAction)_statespace.AllPossibleActions[i];
                for (int j = i; j < _possible_actions_index.Count; j += ((VrpStateSpace)_statespace).Requests.Count * 2)
                    if (i != j)
                    {
                        VrpAction action_j = (VrpAction)_statespace.AllPossibleActions[j];

                        action_i.SetBackwardDependency(action_j);
                        action_j.SetBackwardDependency(action_i);
                    }
            }
        }

		/// <summary>
        /// This Method evaluates if the action2 is dependent from action1
		/// </summary>
		/// <param name="_possible_actions">An Action</param>
		/// <param name="p1">An Action</param>
		/// <returns>A  bool</retutns>
		protected override bool IsActionBackwardDependent(Action action1,Action action2)
		{
            VrpRequestAction vrpaction1 = (VrpRequestAction)action1;
            VrpRequestAction vrpaction2 = (VrpRequestAction)action2;
            
            
            if(vrpaction1.VehicleIndex != vrpaction2.VehicleIndex && vrpaction1.RequestIndex == vrpaction2.RequestIndex)
                return true;
            else
                return false;
        }



        public VrpState GetLastStateFromVehicle(int vehicle_index)
        {
            VrpState state = this;
            while (state != null && state.Action != null)
            {
                if (((VrpAction)state.Action).VehicleIndex == vehicle_index)
                    return state;
                state = (VrpState)state.PreviousState;
            }
            return null;
        }

        public override string ToString()
        {

            string actions_prev = "";
            State state = this;
            while (state.Action != null) {
                actions_prev += state.Action + ((VrpState)state).CurrTime[((VrpAction)state.Action).VehicleIndex].ToString("HH:mm") + " ";
                state = state.PreviousState;
            }
            if (_action == null)
                return "InitialState";
            else
                return _action.ToString() + " Depth " + _depth_state + " TD " + Math.Floor(CurrTraveldistanceAll) + " TT " + string.Format("{0:0.0}", CurrTraveltimeAllSecs / 60f) + "{" + actions_prev.TrimEnd() + "}";
        }


        #region clone logic
        /// <summary>
        /// Creats a deep clone of the current state (also previous states are cloned). 
        /// Only the Next_States List is dismissed.
        /// </summary>
        /// <returns>A new object that is a deep copy of this instance.</returns>
        /// <filterpriority>2</filterpriority>
        public override Object Clone()
        {
            VrpState clone_state = (VrpState)CloneShallow();
                
            //clone also the previous state if possible
            if (this._prev_state != null)
            {
                clone_state._prev_state = (VrpState)((VrpState)this._prev_state).Clone();
                clone_state._prev_state.LastQueriedNextState = clone_state;
            }

            return clone_state;
        }

        /// <summary>Creates a new object that is a copy of the current instance.</summary>
        /// <returns>A new object that is a copy of this instance.</returns>
        /// <filterpriority>2</filterpriority>
        public override State CloneShallow()
        {
            VrpState clone_state = new VrpState(_time_at_curr_vehiclepos.Length);
            clone_state._curr_traveltime_all_secs = this._curr_traveltime_all_secs;
            clone_state._curr_traveldistance_all = this._curr_traveldistance_all;
            //clone_state._poss_time_shift_back = this._poss_time_shift_back;
            //clone_state._poss_time_shift_forward = this._poss_time_shift_forward;
            clone_state._vehicle_utilization = (int[])this._vehicle_utilization.Clone();
            clone_state._last_served_request_index = (int[])this._last_served_request_index.Clone();
            clone_state._last_served_request_ispickup = (bool[])this._last_served_request_ispickup.Clone();
            clone_state._curr_vehicle_pos = (Vector2f[])this._curr_vehicle_pos.Clone();
            clone_state._time_at_curr_vehiclepos = (DateTime[])this._time_at_curr_vehiclepos.Clone();
            clone_state._depth_state = this._depth_state;
            clone_state._possible_actions_index = this._possible_actions_index.Clone();
            clone_state._prev_state = this._prev_state;
            clone_state._action = this._action;
            clone_state._lastqueried_next_state = null;
            clone_state._next_states = null;
            clone_state._statespace = _statespace;
            if (this._taboolist != null)
                clone_state._taboolist = (uint[])this._taboolist.Clone();

            return clone_state;
        }

        #region Vehicle Dependent Cloning
        /// <summary>
        /// clones a state for the given vehicle index (only if state is of this vehicle index)
        /// the previous state is not cloned, its taken as original
        /// </summary>
        /// <param name="state_to_copy"></param>
        /// <param name="vehicle_index"></param>
        /// <returns></returns>
        public VrpState CloneVehicleDependent(int vehicle_index, VrpStateSpace new_statespace, List<Request> old_request_list, List<Request> new_request_list)
        {
            if (vehicle_index < 0 || vehicle_index > _vehicle_utilization.Length - 1)
                throw new Exception("cannot clone for this vehicle as vehicle not given");

            VrpState clone_state = new VrpState(1);
            //clone_state._poss_time_shift_back = _poss_time_shift_back;
            //clone_state._poss_time_shift_forward = _poss_time_shift_forward;
            clone_state._curr_traveltime_all_secs = _curr_traveltime_all_secs;
            clone_state._time_at_curr_vehiclepos[0] = _time_at_curr_vehiclepos[vehicle_index];
            clone_state._curr_vehicle_pos[0] = _curr_vehicle_pos[vehicle_index];
            if (_last_served_request_index[vehicle_index] >= 0)
                clone_state._last_served_request_index[0] = new_request_list.IndexOf(old_request_list[_last_served_request_index[vehicle_index]]);
            else
                clone_state._last_served_request_index[0] = -1;
            clone_state._last_served_request_ispickup[0] = _last_served_request_ispickup[vehicle_index];
            clone_state._vehicle_utilization[0] = _vehicle_utilization[vehicle_index];
            clone_state._prev_state = ((VrpState)_prev_state).GetLastStateFromVehicle(vehicle_index);
            clone_state._action = GetClonedVehicleDependentAction((VrpAction)_action, vehicle_index,new_statespace, old_request_list, new_request_list);
            clone_state._depth_state = _depth_state;
            clone_state._statespace = new_statespace;


            CloneVehicleDependentNextStates(vehicle_index, clone_state, new_statespace, old_request_list, new_request_list);
            
            return clone_state;
        }
		/// <summary>
		/// Method GetClonesAction
		/// </summary>
		/// <param name="_action">An Action</param>
		/// <param name="new_inital_state">A  VrpState</param>
		/// <returns>An Action</retutns>
        private VrpAction GetClonedVehicleDependentAction(VrpAction org_action, int vehicle_index, VrpStateSpace new_statespace, List<Request> old_request_list, List<Request> new_request_list)
		{
            if (org_action is VrpRequestAction)
            {
                Request r = old_request_list[((VrpRequestAction)org_action).RequestIndex];
                int new_index_of_request = new_request_list.IndexOf(r);
                if (new_index_of_request < 0)
                    return null;
                int index_new_action = new_index_of_request * 2 + ((((VrpRequestAction)org_action).IsPickup) ? 0 : 1);
                return (VrpRequestAction)new_statespace.AllPossibleActions[index_new_action];
            }
            else if (org_action is VrpStopAction)
            {
                int first_custom_action = new_statespace.Requests.Count * 2 * new_statespace.Vehicles.Count;
                for (int i = first_custom_action; i < new_statespace.AllPossibleActions.Count; i++)
                {
                    if (!(new_statespace.AllPossibleActions[i] is VrpStopAction))
                        continue;
                    
                    VrpStopAction action = (VrpStopAction)new_statespace.AllPossibleActions[i];

                    if (((VrpStopAction)org_action).VehicleIndex != vehicle_index)
                        continue;
                    if (((VrpStopAction)org_action).StopPosition != action.StopPosition)
                        continue;
                    if (((VrpStopAction)org_action).StopTime != action.StopTime)
                        continue;

                    return action;
                }
                return null;
            }
            else 
            {
                throw new Exception("action type unkown");
            }
		}

        /// <summary>
        /// call this function to clone all next_states, possible_actions, taboo_list
        /// </summary>
        /// <param name="vehicle_index">the vehicle to use, only states concerning this vehicle will be cloned</param>
        /// <param name="clone_state">this is the state where the clones will be stored at</param>
        private void CloneVehicleDependentNextStates(int vehicle_index, VrpState clone_state, VrpStateSpace new_statespace, List<Request> old_request_list, List<Request> new_request_list)
        {
              
            //clone_state._possible_actions_index = _possible_actions_index.Clone();
            clone_state._possible_actions_index = new Logicx.Utilities.MemoryEfficientSequentialIntegerList(new_statespace.AllPossibleActions.Count);


            if (HasTabooActions)
            {
                throw new Exception("taboo list clone not handled yet");
                clone_state._taboolist = new uint[_taboolist.Length];

            }

            //int index_new_state_possible_actions = 0;
            for (int i = 0; i < _possible_actions_index.Count; i++)
            {
                //enable fast iteration through list
                if (i % 32 == 0 && _possible_actions_index.List[(int)Math.Floor(i / 32f)] == 0)
                {
                    i += 31;
                    continue;
                }
                if (!_possible_actions_index.Contains(i))
                    continue;

                //clear possible actions
                VrpAction action_i = (VrpAction)_statespace.AllPossibleActions[i];

                //is action from to be cloned vehicle?
                if (action_i.VehicleIndex == vehicle_index)
                {
                    VrpAction action_cloned = GetClonedVehicleDependentAction(action_i, vehicle_index, new_statespace, old_request_list, new_request_list);
                    if (action_cloned != null)
                    {
                        int index_action_cloned = new_statespace.AllPossibleActions.IndexOf(action_cloned);
                        clone_state._possible_actions_index.Add(index_action_cloned);

                        if (HasTabooActions)
                            if (IsTabooAction(i))
                            {
                                throw new Exception("taboo list clone not handled yet");
                                int a = 120;//clone_state.SetActionTaboo(index_new_state_possible_actions);
                            }
                    }
                }
            }
        }
        #endregion

        #endregion

        #region Attributes
        protected DateTime[] _time_at_curr_vehiclepos;
        protected Vector2f[] _curr_vehicle_pos;
        protected int[] _last_served_request_index;
        protected bool[] _last_served_request_ispickup;
        protected int[] _vehicle_utilization;
        protected float _curr_traveldistance_all = 0;
        protected float _curr_traveltime_all_secs = 0;
        //protected int _poss_time_shift_back = 0;
        //protected int _poss_time_shift_forward = 0;
        #endregion
    }
}
