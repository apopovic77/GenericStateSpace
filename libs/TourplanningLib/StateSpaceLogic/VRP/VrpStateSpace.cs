using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Geo.Geometries;
using Logicx.Gis.Routeplanning.RouteplannerLogic;
using Logicx.Optimization.GenericStateSpace;
using Logicx.Optimization.Tourplanning.NearestNeighbour;
using MathLib;
using Logicx.Utilities;
using Action = Logicx.Optimization.GenericStateSpace.Action;


namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{
    public class VrpStateSpace : StateSpace
    {
        public VrpStateSpace(List<Vehicle> vehiles, List<Request> requests, DateTime start_time, Routeplanner planner)
        {
            _vehicles = vehiles;
            _requests = requests;
            _count_vehicles = _vehicles.Count;
            _count_requesting_persons = requests.Count;
            _start_time = start_time;

            _planner = planner;

            //cache the point in poly validation,
            //this is needed for the assertvehicledservicedarea CONSTRAINT
            BuildCacheServicedAreaCheck();
        }

        #region StateSpace Splitting
        /// <summary>
		/// Method SplitSpace
		/// </summary>
		/// <returns>A  VrpStateSpace</retutns>
		public virtual VrpStateSpace SplitSpace(int vehicle_index)
		{
            return SplitSpace(vehicle_index, Requests);
		}

        /// <summary>
        /// Method SplitSpace
        /// </summary>
        /// <returns>A  VrpStateSpace</retutns>
        public virtual VrpStateSpace SplitSpace(int vehicle_index, List<Request> requests)
        {
            if (vehicle_index > Vehicles.Count)
                throw new Exception("cannot split statepace, as vehicle not existent");

            List<Vehicle> vehicles = Vehicles.GetRange(vehicle_index, 1);
            VrpStateSpace statespace_i = new VrpStateSpace(vehicles, requests, _start_time, _planner);
            statespace_i.InitializeFirstState();

            //return immediatelly if there are no 
            if (_custom_actions_count == 0)
                return statespace_i;

            int first_custom_action = _requests.Count * 2 * Vehicles.Count;
            for (int i = first_custom_action; i < _all_possible_actions.Count; i++)
            {
                if (!(_all_possible_actions[i] is VrpStopAction))
                    throw new Exception("cannot handle splitspace, unknown custom action type");

                VrpStopAction vrpadvstopaction = (VrpStopAction)_all_possible_actions[i];
                if (vrpadvstopaction.VehicleIndex != vehicle_index)
                    continue;

                VrpStopAction action_clone = (VrpStopAction)vrpadvstopaction.Clone();
                action_clone.VehicleIndex = 0;
                statespace_i.AddCustomAction(action_clone);
            }
            return statespace_i;


        }
        #endregion

        #region ORG Action determination
        public virtual VrpRequestAction GetOrgRequestAction(Request r,int org_vehicle_index,bool ispickup) {

            int org_request_index = _requests.IndexOf(r);
            int index_org_ref_action = _requests.Count * 2 * org_vehicle_index + org_request_index * 2 + ((ispickup) ? 0 : 1);
            VrpRequestAction org_ref_action = (VrpRequestAction)_all_possible_actions[index_org_ref_action];
            return org_ref_action;
        
        }


        public virtual VrpAction GetOrgVrpAction(VrpAction action_clone, List<Request> requests, int org_vehicle_index)
        {
            if (action_clone is VrpRequestAction)
            {
                VrpRequestAction vrp_action_clone = (VrpRequestAction)action_clone;

                //get the org request of the action
                Request r = requests[vrp_action_clone.RequestIndex];

                //get the org ref action, also die orignal action im statespace
                return this.GetOrgRequestAction(r, org_vehicle_index, vrp_action_clone.IsPickup);            
            }
            else if (action_clone is VrpStopAction)
            {
                VrpStopAction stop_action_clone = (VrpStopAction)action_clone;
                int first_custom_action = _requests.Count * 2 * _vehicles.Count;
                for (int i = first_custom_action; i < _all_possible_actions.Count; i++)
                {
                    if (!(_all_possible_actions[i] is VrpStopAction))
                        continue;

                    VrpStopAction org_action = (VrpStopAction)_all_possible_actions[i];

                    if (org_action.VehicleIndex != org_vehicle_index)
                        continue;
                    if (org_action.StopPosition != stop_action_clone.StopPosition)
                        continue;
                    if (org_action.StopTime != stop_action_clone.StopTime)
                        continue;

                    return org_action;
                }
                throw new Exception("org action not existing");
            }
            else
                throw new Exception("unknown action type");
        }
        #endregion

        #region Caching Strategies
        /// <summary>
        /// Method BuildCacheServicedAreaCheck
        /// </summary>
        private void BuildCacheServicedAreaCheck()
        {
            _cache_servicedarea_pointinpolycheck = new List<List<bool>>(_vehicles.Count);
            for (int vehicle_index = 0; vehicle_index < _vehicles.Count; vehicle_index++)
            {
                _cache_servicedarea_pointinpolycheck.Add(null);
                // continue if we dont have to do any service area check for this vehicle
                //this means the vehicles serves all requests
                if (_vehicles[vehicle_index].ServicedArea == null)
                    continue;

                // this vehicle has polygons attached, which means we have to check for
                // request locs if they are serviced by this vehicle or not
                // create a list of bools that store the info if the vehicles serves the request or not
                // for each request we have two booleans to be validated, one for pickup and one for destination
                // if both pickup and destinations point in poly check succeds the request can be seen
                // served (simply said :-)) 
                _cache_servicedarea_pointinpolycheck[vehicle_index] = new List<bool>(_requests.Count);
                for (int request_index = 0; request_index < _requests.Count; request_index++)
                {
                    object serviced = false;
                    object notserviced = false;
                    object pickupservice = false;
                    object deliveryservice = false;
                    //check with every polygon that is attached to the vehicle
                    for (int polygon_index = 0; polygon_index < _vehicles[vehicle_index].ServicedArea.Length; polygon_index++)
                    {
                        //get the current polygon from the vehicle
                        SimplePolygon poly = _vehicles[vehicle_index].ServicedArea[polygon_index];
                        //do the contraint check
                        switch (_vehicles[vehicle_index].ServicedAreaType[polygon_index])
                        {
                            case Vehicle.ServicedAreaTypes.Serviced:

                                if (poly.PointInPolygon(_requests[request_index].FromUtm) &&
                                    poly.PointInPolygon(_requests[request_index].ToUtm))
                                {
                                    if (serviced != null)
                                        serviced = (bool)serviced | true;
                                    else
                                        serviced = true; //es wird fix geserviced
                                }
                                else
                                {
                                    if (serviced != null)
                                        serviced = (bool)serviced | false;
                                    else
                                        serviced = false;
                                }

                                break;
                            case Vehicle.ServicedAreaTypes.NotServiced:

                                if (poly.PointInPolygon(_requests[request_index].FromUtm) ||
                                    poly.PointInPolygon(_requests[request_index].ToUtm))
                                {
                                    if (notserviced != null)
                                        notserviced = (bool)notserviced | true;
                                    else
                                        notserviced = true; //es wird fix geserviced
                                }
                                else
                                {
                                    if (notserviced != null)
                                        notserviced = (bool)notserviced | false;
                                    else
                                        notserviced = false; //es wird fix geserviced
                                }
                                break;
                            case Vehicle.ServicedAreaTypes.PickupServiced:

                                if (poly.PointInPolygon(_requests[request_index].FromUtm) &&
                                    !poly.PointInPolygon(_requests[request_index].ToUtm))
                                {
                                    if (pickupservice != null)
                                        pickupservice = (bool)pickupservice | true;
                                    else
                                        pickupservice = true; //es wird fix geserviced
                                }
                                else
                                {
                                    if (pickupservice != null)
                                        pickupservice = (bool)pickupservice | false;
                                    else
                                        pickupservice = false; //es wird fix geserviced
                                }
                                break;
                            case Vehicle.ServicedAreaTypes.DeliveryServiced:

                                if (!poly.PointInPolygon(_requests[request_index].FromUtm) &&
                                     poly.PointInPolygon(_requests[request_index].ToUtm))
                                {
                                    if (deliveryservice != null)
                                        deliveryservice = (bool)deliveryservice | true;
                                    else
                                        deliveryservice = false; //es wird fix geserviced
                                }
                                else
                                {
                                    if (deliveryservice != null)
                                        deliveryservice = (bool)deliveryservice | false;
                                    else
                                        deliveryservice = false; //es wird fix geserviced
                                }
                                break;
                        }
                    }
                    bool res_serviced = false;
                    //setze den request als serviciert wenn er serviced gesetzt wurde
                    if (serviced != null) res_serviced = (bool)serviced;
                    //auch wenn er vorher als true gesetzt wurde wir er fix auf nicht serviciert gesetzt wenn notservied == true ergab
                    if (notserviced != null) res_serviced = !(bool)notserviced;
                    //pickup und delivery service spielt keine rolle wenn not serviced auf true gesetzt wurde, das ist das letzte urteil
                    //wenn jedoch false ist ohne das expliziert durch not serviced oder serviced auf false gesetzt wurde
                    //dann spielt pickupservice oder deliveryservice eine rolle
                    if (pickupservice != null && (!res_serviced && serviced == null && notserviced == null)) res_serviced = (bool)pickupservice;
                    if (deliveryservice != null && (!res_serviced && serviced == null && notserviced == null)) res_serviced = (bool)deliveryservice;
                    //wenn pickupservice und deliveryservice gesetzt wurden dann ist der request nur dann servisiert
                    //wenn beide true ergaben
                    if (pickupservice != null && deliveryservice != null && res_serviced) res_serviced = (bool)pickupservice & (bool)deliveryservice;

                    //cache the evaluated bool value
                    _cache_servicedarea_pointinpolycheck[vehicle_index].Add(res_serviced);
                }
            }
        }
        #endregion

        #region insertion opitmal logic

        /// <summary>
        /// inserts a request into the given statespace
        /// </summary>
        /// <param name="state">the state where we are inserting</param>
        /// <param name="r">the request we want to be inserted</param>
        /// <returns>the new state we the request inserted, it may return null if the insertion is not possible</returns>
        public State InsertRequestOptimal(State state, int request_index)
        {
            //save org state
            State org_state = state;

            //get all actions
            List<Action> actions = new List<Action>();
            while (state.Action != null)
            {
                actions.Add(state.Action);
                state = state.PreviousState;
            }
            state = org_state;

            //start alg with the creation of an new optimal state
            State new_optimal_state = null;
            // run for each vehicle
            for (int vehicle_index = 0; vehicle_index < _vehicles.Count; vehicle_index++)
            {
                State next_best_state = InsertRequestOptimal(state, actions, request_index, vehicle_index);
                if (next_best_state != null && (new_optimal_state == null || new_optimal_state.CurrentTargetValue > next_best_state.CurrentTargetValue))
                    new_optimal_state = next_best_state;
            }

            //return the final result, i may be possible that this is now null
            return new_optimal_state;
        }

        public State InsertRequestOptimal(State state, int request_index, int vehicle_index)
        {
            //save org state
            State org_state = state;
            //get all actions
            List<Action> actions = new List<Action>();
            while (state.Action != null)
            {
                actions.Add(state.Action);
                state = state.PreviousState;
            }
            state = org_state;

            //insert the request optimal
            return InsertRequestOptimal(state, actions, request_index, vehicle_index);
        }

        private State InsertRequestOptimal(State state, List<Action> actions, int request_index, int vehicle_index)
        {

            //the the inital state which we need to build a new optimal state
            State new_state = _initial_state;

            //get the pickup action for this
            int index_pickup_action = _requests.Count * 2 * vehicle_index + request_index * 2;
            VrpRequestAction pickup_action = (VrpRequestAction)_all_possible_actions[index_pickup_action];

            //get the pickup action for this
            int index_delivery_action = _requests.Count * 2 * vehicle_index + request_index * 2 + 1;
            VrpRequestAction delivery_action = (VrpRequestAction)_all_possible_actions[index_delivery_action];

            //check for trivial case
            if (actions.Count == 0)
            {
                new_state = NextState(_initial_state, pickup_action);
                if (state == null)
                    return null;
                new_state = NextState(new_state, delivery_action);
                return new_state;
            }

            //do it for the non trivial case
            State new_optimal_state = null;
            State saved_new_state = new_state;
            for (int j = actions.Count - 1; j >= 0; j--)
            {

                //get the new state
                new_state = NextState(saved_new_state, pickup_action);

                //check if action is possible
                if (new_state != null)
                {
                    //try all delivery action position possiblities
                    int start_action_index = j;
                    for (int delivery_insertion_index = 0; delivery_insertion_index <= start_action_index + 1; delivery_insertion_index++)
                    {
                        bool failed_at_given_delivery_action;
                        //create a new final state that has the request inserted
                        State new_final_state = InsertionDelivery(new_state, actions, start_action_index, delivery_action, delivery_insertion_index, out failed_at_given_delivery_action);

                        //it does not make sense to continue our search if we cannot insert the delivery immediatelly
                        //i dont think that this should be possible at a later stage
                        if (failed_at_given_delivery_action)
                            break;

                        //save if its current target value is smaller
                        if (new_final_state != null && (new_optimal_state == null || new_optimal_state.CurrentTargetValue > new_final_state.CurrentTargetValue))
                            new_optimal_state = new_final_state;
                    }
                }

                //get the new state
                try
                {
                    WithTimeContraction = true;
                    saved_new_state = NextState(saved_new_state, actions[j]);
                }
                finally 
                {
                    WithTimeContraction = false;
                }
            }

            //get the new state
            new_state = NextState(saved_new_state, pickup_action);
            if (new_state != null)
            {
                new_state = NextState(new_state, delivery_action);
                //save if its current target value is smaller
                if (new_state != null && (new_optimal_state == null || new_optimal_state.CurrentTargetValue > new_state.CurrentTargetValue))
                    new_optimal_state = new_state;
            }

            return new_optimal_state;
        }


        private State InsertionDelivery(State new_state, List<Action> actions, int start_action_index, Action delivery_action, int delivery_insertion_index, out bool failed_at_given_delivery_action)
        {
            failed_at_given_delivery_action = false;
            int curr_step_action_index = 0;
            for (int j = start_action_index; j >= 0; j--)
            {
                //add the pickup request
                if (delivery_insertion_index == curr_step_action_index)
                {
                    //get the new state
                    new_state = NextState(new_state, delivery_action);

                    //check if action is possible
                    if (new_state == null)
                    {
                        failed_at_given_delivery_action = true;
                        return null;
                    }
                }

                try
                {
                    WithTimeContraction = true;
                    //get the new state
                    new_state = NextState(new_state, actions[j]);
                }
                finally
                {
                    WithTimeContraction = false;
                }

                
                
                //check if action is possible
                if (new_state == null)
                    return null;

                curr_step_action_index++;
            }

            if (delivery_insertion_index == curr_step_action_index)
                new_state = NextState(new_state, delivery_action);

            return new_state;
        }
        #endregion

        #region helper methods
        /// <summary>
        /// this method returns all requests that are served from the given vehicle
        /// </summary>
        /// <param name="vehicle_index">the vehicle that serves the requests you want to know</param>
        /// <returns>the requests which are served by the given vehicle</returns>
        public List<Request> GetServedRequests(VrpState org_state, int vehicle_index)
        {
            //save org state to a new var, so that the ref of the org state is not overriden
            VrpState last_state = org_state;
            //create the list to be returned
            List<Request> requests = new List<Request>();
            //run through all states
            while (last_state != null && last_state.Action != null)
            {
                if (last_state.Action is VrpRequestAction)
                {
                    VrpRequestAction action = (VrpRequestAction)last_state.Action;
                    if (action.VehicleIndex == vehicle_index && action.IsPickup)
                        requests.Add(_requests[action.RequestIndex]);
                }

                last_state = (VrpState)last_state.PreviousState;
            }
            return requests;
        }


        public override void AddCustomAction(Action action)
        {
            base.AddCustomAction(action);

            //set dependency for custom action
            //needed for forward constraint checking
            if (action is VrpStopAction)
            {
                //no need to set backward dependency

                //run through all actions from vehicle of the stop action and set a forward dependency
                VrpStopAction stopaction = ((VrpStopAction)action);
                for (int i = Requests.Count * 2 * stopaction.VehicleIndex; i < Requests.Count * 2 * stopaction.VehicleIndex + Requests.Count * 2; i++)
                {
                    Action action_i = _all_possible_actions[i];
                    action_i.SetForwardDependency(action);
                }

                ////what we need to set are forward dependencies
                ////these are need for forward constraint checks
                //for (int i = 0; i < _all_possible_actions.Count; i++)
                //{
                //    Action action_i = _all_possible_actions[i];
                //    if (action_i is VrpStopAction)
                //    {
                //        continue;
                //    }
                //    else if (action_i is VrpRequestAction)
                //    {
                //        if (((VrpRequestAction)action_i).VehicleIndex == ((VrpStopAction)action).VehicleIndex)
                //            action_i.SetForwardDependency(action);
                //    }
                //    else
                //        throw new Exception("Action type unknown");
                //}

            }
            else
            {
                throw new Exception("Action type unknown");
            }
        }

        /// <summary>
        /// Method GetRequestPos
        /// </summary>
        /// <param name="sep_lists">A  VrpAction</param>
        /// <returns>A  Vector2f</retutns>
        private Vector2f GetRequestPos(VrpRequestAction action)
        {
            if (action.IsPickup)
                return _requests[action.RequestIndex].FromUtm;
            else
                return _requests[action.RequestIndex].ToUtm;
        }
        #endregion

        #region clone logic
        /// <summary>
        /// this method splits the statespace into seperate parts
        /// usually a statesapce can consist of multiple vehicle
        /// if you have such a statespace and if you want to have at substatespace
        /// which is only aware of one vehicle, than you can call this medhod
        /// </summary>
        /// <param name="state">the state wich implicitly defines the statespace</param>
        /// <param name="vehicle_index">the vehicle_index of the vehicle you want to extract</param>
        /// <param name="vrp_statespace_v_given">returns the statespace frame with only one vehicle and the requests which are served by this vehicle</param>
        /// <param name="vrp_finalstate_v_given">returns the new state that is only aware of actions concerning the specified vehicle</param>
        public void Split(VrpState org_state, int vehicle_index, out VrpStateSpace vrp_statespace_split, out VrpState vrp_finalstate_split)
        {
            if (vehicle_index < 0 || vehicle_index > _vehicles.Count - 1)
                throw new Exception("cannot clone for this vehicle as vehicle not given");

            //create list of all served requests
            List<Request> vehicle_served_requests = GetServedRequests(org_state, vehicle_index);

            //clone the statespace, we may have less vehicles and less requests now
            vrp_statespace_split = SplitSpace(vehicle_index, vehicle_served_requests);

            //clone and split the state itself
            vrp_finalstate_split = SplitState(org_state, vehicle_index, vrp_statespace_split);
        }

        private VrpState SplitState(VrpState org_state, int vehicle_index, VrpStateSpace clone_vrp_statespace)
        {
            if (vehicle_index < 0 || vehicle_index > org_state.VehicleUtilization.Length - 1)
                throw new Exception("cannot clone for this vehicle as vehicle not given");

            //create a new inital state for the sub statespace
            VrpState cloned_initial_state = (VrpState)clone_vrp_statespace._initial_state;

            //create a var for the storage of the first state getting cloned
            VrpState clone_state_result = null;

            //start cloning states
            //clone the state 
            VrpState last_clone = org_state.GetLastStateFromVehicle(vehicle_index).CloneVehicleDependent(vehicle_index, clone_vrp_statespace, _requests, clone_vrp_statespace._requests);

            while (last_clone.PreviousState != null)
            {
                // here we have in the previous state the org state to copy
                VrpState org_prev_state = (VrpState)last_clone.PreviousState;

                //Create a clone of the previous state and assign it to the clone
                last_clone.PreviousState = org_prev_state.GetLastStateFromVehicle(vehicle_index).CloneVehicleDependent(vehicle_index, clone_vrp_statespace, _requests, clone_vrp_statespace._requests);

                //save the first cloned state
                if (clone_state_result == null)
                    clone_state_result = last_clone;

                //save last_last_clone as last_queried clone
                last_clone.PreviousState.LastQueriedNextState = last_clone;

                //adjust last_clone for while loop
                last_clone = (VrpState)last_clone.PreviousState;
            }

            ////create the inital_state for the new sub state
            //VrpState initial_state = new VrpState(1);
            ////store depot position to first state
            //initial_state.CurrVehiclePos[0] = _vehicles[vehicle_index].DepotUtm;
            ////set start time
            //initial_state.CurrTime[0] = _start_time;
            //((VrpState)_initial_state).CloneShallowNextStates(vehicle_index, initial_state);
            //set the new inital_state clone as previous
            last_clone.PreviousState = cloned_initial_state;
            last_clone.CurrTraveldistanceAll = ((Vector2f)cloned_initial_state.CurrVehiclePos[0] - last_clone.CurrVehiclePos[0]).GetLen();
            //do update of travel distances
            //for that we have to iterate back
            VrpState next_state = (VrpState)last_clone.LastQueriedNextState;
            VrpState curr_state = last_clone;
            int depth_state = 1;
            do
            {

                //adjust travel dist
                float travel_dist = ((Vector2f)next_state.CurrVehiclePos[0] - curr_state.CurrVehiclePos[0]).GetLen();
                next_state.CurrTraveldistanceAll += curr_state.CurrTraveldistanceAll + travel_dist;

                //Adjust depth state
                curr_state.DepthState = depth_state;
                depth_state++;

                curr_state = next_state;
                next_state = (VrpState)next_state.LastQueriedNextState;

            } while (next_state != null);

            clone_state_result.DepthState = depth_state;
            return clone_state_result;
        }
        #endregion

        #region abstract members from the base class
        public override void InitializeFirstState()
        {
            if (_initial_state != null)
                return;

            if (_vehicles.Count == 0)
                throw new Exception("StateSpace: Vehicle Count == 0, this cannot be!!!!!");

            //create the first_state object
            VrpState first_state = new VrpState(_vehicles.Count);

            //prev state is null, as this is the first state
            first_state.PreviousState = null;

            //Create the actions list
            _all_possible_actions = new List<Action>(_vehicles.Count * _requests.Count * 2);

            //set this as statespace
            first_state.StateSpace = this;

            //set cap size for state possible actions
            first_state.PossibleActionsIndex = new Logicx.Utilities.MemoryEfficientSequentialIntegerList(_vehicles.Count * _requests.Count * 2);

            //Build possible next actions List
            for (int vehicle_index = 0; vehicle_index < _vehicles.Count; vehicle_index++)
            {
                //store depot position to first state
                first_state.CurrVehiclePos[vehicle_index] = _vehicles[vehicle_index].CurrentPositionUtm;

                //set start time
                first_state.CurrTime[vehicle_index] = _start_time;
                for (int request_index = 0; request_index < _requests.Count; request_index++)
                {
                    //füge eine neue action in die liste ein
                    VrpRequestAction action_pickup = new VrpRequestAction(vehicle_index, request_index, true);
                    _all_possible_actions.Add(action_pickup);
                    first_state.AddPossibleNextAction(action_pickup);

                    //create the delivery action
                    VrpRequestAction action_delivery = new VrpRequestAction(vehicle_index, request_index, false);
                    _all_possible_actions.Add(action_delivery);
                }
            }

            //build action dependencies, this is needed for performance improvement
            first_state.SetActionDependencies();

            //set first as initial
            _initial_state = first_state;
        }

        /// <summary>
        /// Method CreateNewState
        /// </summary>
        /// <param name="state">A  State</param>
        /// <param name="actions">A  List<Action></param>
        /// <param name="possible_actions_this_node">A  List<Action></param>
        /// <returns>A  State</retutns>
        protected override State CreateNewState(State prev_state, Action action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index_this_node)
        {

            if (action is VrpRequestAction)
            {
                return CreateNewStateRequestAction(prev_state, action, possible_actions_index_this_node);
            }
            else if (action is VrpStopAction)
            {
                return CreateNewStateStopAction((VrpState)prev_state, (VrpStopAction)action, possible_actions_index_this_node);
            }
            else
            {
                throw new Exception("Unknown Action type");
            }


        }

        /// <summary>
        /// Method CreateNewState
        /// </summary>
        /// <param name="state">A  State</param>
        /// <param name="actions">A  List<Action></param>
        /// <param name="possible_actions_this_node">A  List<Action></param>
        /// <returns>A  State</retutns>
        protected State CreateNewStateRequestAction(State prev_state, Action action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index_this_node)
        {
            TimeSpan ts;
            VrpState vrp_prev_state = (VrpState)prev_state;
            VrpRequestAction vrp_action = (VrpRequestAction)action;
            //check if new possible action becomes available
            UpdateNewPossibleActions(action, possible_actions_index_this_node);

            //create the new vrpstate obj
            VrpState vrp_new_state = new VrpState(_vehicles.Count, vrp_prev_state, (VrpRequestAction)action, possible_actions_index_this_node, this);

            ////update state info --> curr vehicle pos --> curr_time_at_pos --> new overall traveldistance
            //this is an separate structure, as vehicles start here from the depot
            Request r = _requests[vrp_action.RequestIndex];
            Vector2f pos_dest;

            if (vrp_action.IsPickup)
                //get the actual destination from the request
                pos_dest = r.FromUtm;
            else
                //get the actual destination from the request
                pos_dest = r.ToUtm;

            //set passenger count
            if (vrp_action.IsPickup) vrp_new_state.VehicleUtilization[vrp_action.VehicleIndex] += r.passengers;
            else vrp_new_state.VehicleUtilization[vrp_action.VehicleIndex] -= r.passengers;

            //get the current pos
            Vector2f curr_pos = vrp_prev_state.CurrVehiclePos[vrp_action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = vrp_prev_state.CurrTime[vrp_action.VehicleIndex];

            //set curr pos
            vrp_new_state.CurrVehiclePos[vrp_action.VehicleIndex] = pos_dest;

            //curr_time at pos (needs to be calculated)
            float travel_distance = 0;
            DateTime best_possible_time_at_dest = GetEstimatedTimeAtPos(curr_pos, pos_dest, curr_time, out travel_distance);
            vrp_new_state.CurrTime[vrp_action.VehicleIndex] = best_possible_time_at_dest;

            //update TravelDistanceAll
            vrp_new_state.CurrTraveldistanceAll += travel_distance;

            //check for time constraints
            //if we can be there much earlier than we have to wait.
            //we cant pick him up before wants it, therefor set best possible time be there
            if (r is PickupRequest && vrp_action.IsPickup)
            {
                if (vrp_new_state.CurrTime[vrp_action.VehicleIndex] < r.EPT)
                {
                    if (_with_time_contraction)
                        vrp_new_state = DoTimeContraction(vrp_new_state, r.EPT);
                    else
                        vrp_new_state.CurrTime[vrp_action.VehicleIndex] = r.EPT;
                }

            }
            else if (r is PickupRequest && !vrp_action.IsPickup)
            {
                /*
                 * 
                 * here we can accept the estimated time
                 * therefore we do nothing here
                 * 
                 */
            }
            else if (r is DeliveryRequest && vrp_action.IsPickup)
            {
                if (vrp_new_state.CurrTime[vrp_action.VehicleIndex] < r.EPT)
                {
                    if (_with_time_contraction)
                        vrp_new_state = DoTimeContraction(vrp_new_state, r.EPT);
                    else
                        vrp_new_state.CurrTime[vrp_action.VehicleIndex] = r.EPT;
                }
            }
            else if (r is DeliveryRequest && !vrp_action.IsPickup)
            {
            }
            else
            {
                throw new Exception("Request type unknown");
            }

            //update curr travel time all
            //second poss, use the travel time
            ts = vrp_new_state.CurrTime[vrp_action.VehicleIndex] - vrp_prev_state.CurrTime[vrp_action.VehicleIndex];
            vrp_new_state.CurrTraveltimeAllSecs = vrp_prev_state.CurrTraveltimeAllSecs + (float)ts.TotalSeconds;

            //XBUG CHECK IF THIS IS OK INSERTED ON 25.08.2008 alex
            //here we take into account the service time
            if (vrp_action.IsPickup)
                vrp_new_state.CurrTime[vrp_action.VehicleIndex] += r.ServiceTime;
            else
                vrp_new_state.CurrTime[vrp_action.VehicleIndex] += new TimeSpan(0, 0, Convert.ToInt32((float)r.ServiceTime.TotalSeconds / 4f));

            return vrp_new_state;
        }


        private State CreateNewStateStopAction(VrpState prev_state, VrpStopAction action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index_this_node)
        {

            TimeSpan ts;

            //create the new vrpstate obj
            VrpState vrp_new_state = new VrpState(_vehicles.Count, prev_state, action, possible_actions_index_this_node, this);

            //get the current pos
            Vector2f curr_pos = prev_state.CurrVehiclePos[action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = prev_state.CurrTime[action.VehicleIndex];

            //set curr pos
            vrp_new_state.CurrVehiclePos[action.VehicleIndex] = action.StopPosition;

            //curr_time at pos (needs to be calculated)
            float travel_distance = 0;
            DateTime best_possible_time_at_dest = GetEstimatedTimeAtPos(curr_pos, action.StopPosition, curr_time, out travel_distance);
            vrp_new_state.CurrTime[action.VehicleIndex] = best_possible_time_at_dest;

            //update TravelDistanceAll
            vrp_new_state.CurrTraveldistanceAll += travel_distance;

            //check for time constraints
            //if we can be there much earlier than we have to wait.
            if (vrp_new_state.CurrTime[action.VehicleIndex] < action.EST)
            {
                //if (_with_time_contraction)
                //    vrp_new_state = DoTimeContraction(vrp_new_state, action.EST);
                //else
                vrp_new_state.CurrTime[action.VehicleIndex] = action.EST;
            }

            //update curr travel time all
            //second poss, use the travel time
            ts = vrp_new_state.CurrTime[action.VehicleIndex] - prev_state.CurrTime[action.VehicleIndex];
            vrp_new_state.CurrTraveltimeAllSecs = prev_state.CurrTraveltimeAllSecs + (float)ts.TotalSeconds;


            ////here we take into account the service time
            vrp_new_state.CurrTime[action.VehicleIndex] += action.ServiceTime;


            return vrp_new_state;
        }


        #region TIME CONTRACTION ALG

        public VrpState ReconstructState(VrpState vrp_state_dst, VrpState vrp_state_src, List<Request> requests, int org_vehicle_index)
        {
            if (vrp_state_src == null)
                return vrp_state_dst;


            //build action list for vehicle i
            List<DateTime> curr_time = new List<DateTime>();
            List<Action> action_list = new List<Action>();

            //get actions and currtime list
            VrpState curr_state = vrp_state_src;
            while (curr_state.PreviousState != null)
            {
                action_list.Add(curr_state.Action);
                curr_time.Add(curr_state.CurrTime[((VrpAction)curr_state.Action).VehicleIndex]);
                curr_state = (VrpState)curr_state.PreviousState;
            }

            //build solution state
            //add actions to final solution list
            for (int action_index = action_list.Count - 1; action_index >= 0; action_index--)
            {
                VrpAction action = (VrpAction)action_list[action_index];
                DateTime time = curr_time[action_index];

                VrpAction org_ref_action = this.GetOrgVrpAction(action, requests, org_vehicle_index);

                float service_time = 0;
                if (action is VrpRequestAction)
                {
                    //get the org request of the action
                    Request r = requests[((VrpRequestAction)action).RequestIndex];
                    service_time = (float)r.ServiceTime.TotalSeconds;
                    ////get the org ref action, also die orignal action im statespace
                    //VrpRequestAction org_ref_action = this.GetOrgRequestAction(r, org_vehicle_index, action.IsPickup);
                }
                else if(action is VrpStopAction)
                {
                    service_time = (float)((VrpStopAction)action).ServiceTime.TotalSeconds;
                }
                else 
                { 
                    throw new Exception("action type unknown");
                }

                //build the next state
                vrp_state_dst.LastQueriedNextState = null;
                vrp_state_dst = (VrpState)this.NextState(vrp_state_dst, org_ref_action);

                //set old time value
                vrp_state_dst.CurrTime[org_vehicle_index] = time;
                TimeSpan ts = vrp_state_dst.CurrTime[org_vehicle_index] - ((VrpState)vrp_state_dst.PreviousState).CurrTime[org_vehicle_index];
                vrp_state_dst.CurrTraveltimeAllSecs = ((VrpState)vrp_state_dst.PreviousState).CurrTraveltimeAllSecs + (float)ts.TotalSeconds - service_time;
            }

            return vrp_state_dst;
        }


        public VrpState ReconstructContractedState(VrpState vrp_state_dst, VrpState vrp_state_src, List<Request> requests, int org_vehicle_index)
        {
            if (vrp_state_src == null)
                return vrp_state_dst;

            //build action list for vehicle [org_vehicle_index]
            List<VrpAction> action_list = new List<VrpAction>();

            //get actions and currtime list
            VrpState curr_state = vrp_state_src;
            while (curr_state.PreviousState != null)
            {
                action_list.Add((VrpAction)curr_state.Action);
                curr_state = (VrpState)curr_state.PreviousState;
            }


            //if we set this to true, than the nextstate method, tries to tie up the timewindows

            try
            {
                this.WithTimeContraction = true;
                //add actions to final solution list
                for (int action_index = action_list.Count - 1; action_index >= 0; action_index--)
                {
                    Action org_ref_action = GetOrgVrpAction(action_list[action_index],requests,org_vehicle_index);

                    vrp_state_dst = (VrpState)this.NextState(vrp_state_dst, org_ref_action);
                    if (vrp_state_dst == null)
                        return null;
                }
                return vrp_state_dst;
            }
            finally 
            {
                this.WithTimeContraction = false;
            }
        }


        /// <summary>
		/// Method DoTimeContraction
		/// </summary>
		/// <param name="vrp_new_state">A  VrpState</param>
		/// <param name="r">A  Request</param>
        protected VrpState DoTimeContraction(VrpState vrp_state_org, DateTime new_time)
		{
            //create a clone
            VrpState vrp_state = (VrpState)vrp_state_org.CloneShallow();
            //get the action the new state
            VrpRequestAction vrp_action = (VrpRequestAction)vrp_state.Action;
            //get the best estimate time, this time would be could but is not possible
            //since we might have to to take care of EPT or stuff
            DateTime best_estimated_time = vrp_state.CurrTime[vrp_action.VehicleIndex];
            //the time we want to set, mostly the EPT of the request
            vrp_state.CurrTime[vrp_action.VehicleIndex] = new_time;
            //the time we are contracting
            TimeSpan time_bend = new_time - best_estimated_time;
            //curr vehicle index
            int vehicle_index = ((VrpRequestAction)vrp_state_org.Action).VehicleIndex;

            //start time contraction algorithm

            //1. go for last pickup action
            List<VrpAction> actions;
            VrpState last_pickup_state = GetLastPickupState(vrp_state, out actions);
            if (last_pickup_state.Action == null)
                return vrp_state;

            //2. adjust time bend
            VrpState new_state = (VrpState)last_pickup_state.CloneShallow();
            BendTime(new_state, time_bend);

            //3. check contraints of following states
            new_state = AssertPossibleState(new_state, actions);
            if(new_state != null)
                BendTime(new_state, -_requests[vrp_action.RequestIndex].ServiceTime);
            

            //4. do recursive time contraction for previous pickups too
            if (new_state != null && last_pickup_state.VehicleUtilization[vehicle_index] - _requests[((VrpRequestAction)last_pickup_state.Action).RequestIndex].passengers > 0)
            {
                VrpState contracted_prev_pickupstate = DoTimeContraction(last_pickup_state, last_pickup_state.CurrTime[vehicle_index] + time_bend);

                TimeSpan ts_diff = contracted_prev_pickupstate.CurrTime[vehicle_index] - (last_pickup_state.CurrTime[vehicle_index] + time_bend);
                BendTime(contracted_prev_pickupstate, -ts_diff);
                
                contracted_prev_pickupstate = AssertPossibleState(contracted_prev_pickupstate, actions);
                if (contracted_prev_pickupstate != null)
                {
                    BendTime(contracted_prev_pickupstate, -_requests[vrp_action.RequestIndex].ServiceTime);
                    return contracted_prev_pickupstate;
                }
                else
                    return new_state;

            }
            else if (new_state != null)
            {
                //VrpAction prev_vrp_action = (VrpAction)new_state.PreviousState.Action;
                //VrpAction last_vrp_action =  (VrpAction)new_state.Action;

                //Vector2f from = (prev_vrp_action.IsPickup) ? _requests[prev_vrp_action.RequestIndex].FromUtm : _requests[prev_vrp_action.RequestIndex].ToUtm;
                //Vector2f to = _requests[last_vrp_action.RequestIndex].FromUtm;
                //Routeplan rp = _planner.GetRoutePlan(from,to);
                //TimeSpan rt = rp.TravelTime;

                //DateTime best_possible_pickup_of_previous_state = new_state.CurrTime[vehicle_index] - rt;
                //if (best_possible_pickup_of_previous_state > ((VrpState)new_state.PreviousState).CurrTime[vehicle_index])
                //    //bend pickup
                //    BendTime((VrpState)new_state.PreviousState, best_possible_pickup_of_previous_state - ((VrpState)new_state.PreviousState).CurrTime[vehicle_index]);

                return new_state;
            }
            else
            {
                //time contraction for previous pickups not possible
                return vrp_state;
            }

            //if (vrp_state.LastServedRequestIspickup[vrp_action.VehicleIndex] && ((VrpState)vrp_state.PreviousState).LastServedRequestIndex[vrp_action.VehicleIndex] >= 0)
            //{
            //    //adjust time window of previous pickup request
            //    //as we are to early at the destination and the last action of this vehicle
            //    //was a pickup --> we can go for the last pickup at a later time

            //    //get last state of vehicle from current action
            //    State last_state_of_vehicle = vrp_state.PreviousState;
            //    last_state_of_vehicle.LastQueriedNextState = vrp_state;
            //    while (((VrpAction)last_state_of_vehicle.Action).VehicleIndex != vrp_action.VehicleIndex)
            //    {
            //        State last_next_state = last_state_of_vehicle;
            //        last_state_of_vehicle = last_state_of_vehicle.PreviousState;
            //        last_state_of_vehicle.LastQueriedNextState = last_next_state;
            //    }

            //    //get the request of the last state of this vehicle
            //    Request r_last = _requests[((VrpAction)last_state_of_vehicle.Action).RequestIndex];
            //    VrpState last_vrpstate_of_vehicle = (VrpState)last_state_of_vehicle;

            //    last_vrpstate_of_vehicle.CurrTime[vrp_action.VehicleIndex] = vrp_state.CurrTime[vrp_action.VehicleIndex] - (best_possible_time_at_dest - curr_time);
            //    if (last_vrpstate_of_vehicle.CurrTime[vrp_action.VehicleIndex] > r_last.LPT)
            //        last_vrpstate_of_vehicle.CurrTime[vrp_action.VehicleIndex] = r_last.LPT;

            //    //travel time anpassung
            //    ts = last_vrpstate_of_vehicle.CurrTime[vrp_action.VehicleIndex] - ((VrpState)last_vrpstate_of_vehicle.PreviousState).CurrTime[vrp_action.VehicleIndex];
            //    last_vrpstate_of_vehicle.CurrTraveltimeAllSecs = ((VrpState)last_vrpstate_of_vehicle.PreviousState).CurrTraveltimeAllSecs + (float)ts.TotalSeconds - (float)r_last.ServiceTime.TotalSeconds;

            //    //service time anpassung
            //    //last_vrpstate_of_vehicle.CurrTime[vrp_action.VehicleIndex] += r_last.ServiceTime;

            //    //iterate through next states and adjust travel time all secs
            //    while (last_vrpstate_of_vehicle.LastQueriedNextState != vrp_state)
            //    {
            //        VrpState last_prev_state = last_vrpstate_of_vehicle;
            //        last_vrpstate_of_vehicle = (VrpState)last_vrpstate_of_vehicle.LastQueriedNextState;
            //        r_last = _requests[((VrpAction)last_vrpstate_of_vehicle.Action).RequestIndex];
            //        float ts_rlast;
            //        if (((VrpAction)last_vrpstate_of_vehicle.Action).IsPickup)
            //            ts_rlast = (float)r_last.ServiceTime.TotalSeconds;
            //        else
            //            ts_rlast = (float)r_last.ServiceTime.TotalSeconds / 4f;

            //        //travel time anpassung
            //        ts = last_vrpstate_of_vehicle.CurrTime[vrp_action.VehicleIndex] - last_prev_state.CurrTime[vrp_action.VehicleIndex];
            //        vrp_state.CurrTraveltimeAllSecs = last_prev_state.CurrTraveltimeAllSecs + (float)ts.TotalSeconds - ts_rlast;
            //    }
            //}			
		}
		

		private VrpState AssertPossibleState(VrpState vrp_state, List<VrpAction> actions)
		{
            bool old_val_tc = _with_time_contraction;
            _with_time_contraction = false;
            for (int i = actions.Count - 1; i >= 0; i--)
            {
                vrp_state = (VrpState)NextState(vrp_state, actions[i]);
                if (vrp_state == null)
                    break;
            }
            _with_time_contraction = old_val_tc;
            return vrp_state;
		}
		
		private void BendTime(VrpState vrp_state, TimeSpan time_bend)
		{
            VrpRequestAction vrp_action = (VrpRequestAction)vrp_state.Action;
            //get the request of the last state of this vehicle
            Request r = _requests[((VrpRequestAction)vrp_state.Action).RequestIndex];

            vrp_state.CurrTime[vrp_action.VehicleIndex] = vrp_state.CurrTime[vrp_action.VehicleIndex] + time_bend;
            if (vrp_state.CurrTime[vrp_action.VehicleIndex] > r.LPT)
                vrp_state.CurrTime[vrp_action.VehicleIndex] = r.LPT;

            //travel time anpassung
            TimeSpan ts = vrp_state.CurrTime[vrp_action.VehicleIndex] - ((VrpState)vrp_state.PreviousState).CurrTime[vrp_action.VehicleIndex];
            vrp_state.CurrTraveltimeAllSecs = ((VrpState)vrp_state.PreviousState).CurrTraveltimeAllSecs + (float)ts.TotalSeconds - (float)r.ServiceTime.TotalSeconds;
		}
		private VrpState GetLastPickupState(VrpState vrp_state,out List<VrpAction> actions)
		{

            actions = new List<VrpAction>();
            actions.Add((VrpRequestAction)vrp_state.Action);

            VrpState last_state = (VrpState)vrp_state.PreviousState;
            last_state.LastQueriedNextState = vrp_state;

            if (last_state.Action == null)
                return last_state;

            while ( !(last_state.Action is VrpRequestAction)||
                    ((VrpRequestAction)last_state.Action).VehicleIndex != ((VrpRequestAction)vrp_state.Action).VehicleIndex || 
                    !((VrpRequestAction)last_state.Action).IsPickup
                  )
            {

                actions.Add((VrpAction)last_state.Action);

                State last_next_state = last_state;
                last_state = (VrpState)last_state.PreviousState;
                last_state.LastQueriedNextState = last_next_state;

                if (last_state.Action == null)
                    return last_state;
            }

            return last_state;
        }
        #endregion

        /// <summary>
        /// this method predicts a possible time at the arrivel
        /// therefore this method corresponds to the zuverlässigkeit gebenende method
        /// the better the value here estimated the better the plan.
        /// the current prediction method is based on a simple airline consideration.
        /// here we use an avg speed to estimate the arrivel over the airline
        /// </summary>
        /// <param name="r">Der betroffene Request, welcher Request wird hier angefahren</param>
        /// <param name="curr_pos">Where is the vehicle located now</param>
        /// <param name="pos_dest">What is the desired destination location</param>
        /// <param name="_start_time">what is the time at curr_pos</param>
        /// <returns>returns the estimated time at destination pos</returns>
        protected DateTime GetEstimatedTimeAtPos(Vector2f curr_pos, Vector2f pos_dest, DateTime time_at_curr_pos, out float travel_distance)
        {
            Routeplan rp = _planner.GetRoutePlan(curr_pos, pos_dest);
            travel_distance = rp.TravelDistance;
            return time_at_curr_pos + rp.TravelTime;
        }


        /// <summary>
        /// durch eine aktion kann es sein das es neue actions gibt die zu erfüllen sind.
        /// diese methode sollte gewährleisten das actions für bestimmte situationen eingefügt werden wenn sie möglich werden.
        /// </summary>
        /// <param name="actions">An Action</param>
        /// <param name="possible_actions_this_node">A  List<Action></param>
        protected override void UpdateNewPossibleActions(Action action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index_this_node)
        {
            VrpRequestAction vrpaction = (VrpRequestAction)action;
            if (vrpaction.IsPickup)
            {
                //get delivery action from initial state
                int index_delivery_action = _requests.Count * 2 * vrpaction.VehicleIndex + vrpaction.RequestIndex * 2 + 1;
                //VrpAction new_delivery_action = (VrpAction)_all_possible_actions[index_delivery_action];
                //vrpaction.SetDependency(newaction);
                possible_actions_index_this_node.Add(index_delivery_action);
            }
        }


        #endregion

        #region Constraint Validation Logic

        #region Forward Contraint Checking
        protected override bool AssertForwardConstraints(State state, Action action)
        {
            if (action is VrpRequestAction)
            {
                //get all dependent actions
                List<Action> fwdepactions = action.GetForwardDependentActions();
                if (fwdepactions == null || fwdepactions.Count == 0)
                    return true;

                MemoryEfficientSequentialIntegerList possactions = new MemoryEfficientSequentialIntegerList();
                State new_next_state = CreateNewState(state, action, possactions);

                foreach (Action action_i in fwdepactions)
                {
                    int index_action_i = _all_possible_actions.IndexOf(action_i);
                    new_next_state.PossibleActionsIndex.Add(index_action_i);

                    //dont check for that action was performed already
                    if (!state.PossibleActionsIndex.Contains(index_action_i))
                        continue;

                    _with_forward_constraint_checking = false;
                    VrpState state_test = (VrpState)NextState(new_next_state, action_i);
                    _with_forward_constraint_checking = true;
                    if (state_test == null)
                        return false;
                }

                return true;
            }
            else
                return true;
        }
        #endregion

        #region Backward Constraint checking

        protected override bool AssertBackwardConstraints(State state, Action action)
        {
            if (action is VrpRequestAction)
            {
                if (!AssertBackwardVehicleConstraints((VrpState)state, (VrpRequestAction)action))
                    return false;
                if (!AssertBackwardRequestContraints((VrpState)state, (VrpRequestAction)action))
                    return false;

                return true;
            }
            else if (action is VrpStopAction)
            {
                return AssertBackwardConstraintsStopAction((VrpState)state, (VrpStopAction)action);
            }
            else
                throw new Exception("Action type unknown");
        }

        #region Request Contraint Checking

        /// <summary>
        /// Wird wenn ich diese Action einfüge ein Constraint verletzt,
        /// um dies festzustellen brauche ich nur die letzte action sehen
        /// </summary>
        /// <param name="old_state">The state where we are coming from</param>
        /// <param name="action">the action we are trying to execute</param>
        /// <returns>true -  if action can be performed and no contraint is violated</returns>
        protected virtual bool AssertBackwardRequestContraints(VrpState old_state, VrpRequestAction action)
        {
            Request r = _requests[action.RequestIndex];

            if (r is PickupRequest)
            {
                if (action.IsPickup)
                {

                    return AssertPickupRequest_WartezeitfensterContraint(old_state, action);
                }
                else
                {
                    return AssertPickupRequest_ReisezeitfensterContraint(old_state, action);
                }
            }
            else if (r is DeliveryRequest)
            {
                if (action.IsPickup)
                {

                    return AssertDeliveryRequest_WartezeitfensterContraint(old_state, action);
                }
                else
                {
                    return AssertDeliveryRequest_ReisezeitfensterContraint(old_state, action);
                }
            }
            else
                throw new Exception("Request type currently not supported");
        }

        /// <summary>
        /// Method AssertDeliveryRequest_ReisezeitfensterContraint
        /// </summary>
        /// <param name="old_state">A  VrpState</param>
        /// <param name="action">A  VrpAction</param>
        /// <returns>A  bool</retutns>
        private bool AssertDeliveryRequest_ReisezeitfensterContraint(VrpState old_state, VrpRequestAction action)
        {
            DeliveryRequest r = (DeliveryRequest)_requests[action.RequestIndex];

            //get the actual destination from the request
            Vector2f pos_dest = r.ToUtm;
            //get the current pos
            Vector2f curr_pos = old_state.CurrVehiclePos[action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = old_state.CurrTime[action.VehicleIndex];

            //curr_time at pos (needs to be calculated)
            float travel_distance;
            DateTime estimated_time_at_dest = GetEstimatedTimeAtPos(curr_pos, pos_dest, curr_time, out travel_distance);

            VrpState curr_state = old_state;
            while (((VrpRequestAction)curr_state.Action).RequestIndex != action.RequestIndex)
            {
                curr_state = (VrpState)curr_state.PreviousState;
            }
            DateTime r_time_pickup = curr_state.CurrTime[action.VehicleIndex];
            TimeSpan estimated_ride_time = estimated_time_at_dest - r_time_pickup;

            //check contraint
            if (estimated_time_at_dest < r.LDT && (estimated_ride_time < r.GetMRT(_planner) /*|| _with_time_contraction*/))
                return true;
            else
                return false;
        }

        /// <summary>
        /// Method AssertDeliveryRequest_WartezeitfensterContraint
        /// </summary>
        /// <param name="old_state">A  VrpState</param>
        /// <param name="action">A  VrpAction</param>
        /// <returns>A  bool</retutns>
        private bool AssertDeliveryRequest_WartezeitfensterContraint(VrpState old_state, VrpRequestAction action)
        {
            DeliveryRequest r = (DeliveryRequest)_requests[action.RequestIndex];

            //get the actual destination from the request
            Vector2f pos_dest = r.FromUtm;
            //get the current pos
            Vector2f curr_pos = old_state.CurrVehiclePos[action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = old_state.CurrTime[action.VehicleIndex];

            //curr_time at pos (needs to be calculated)
            float airline_len_meters;
            DateTime estimated_time_at_dest = GetEstimatedTimeAtPos(curr_pos, pos_dest, curr_time, out airline_len_meters);

            //check contraint
            if (estimated_time_at_dest > r.LPT)
                return false;
            else
                return true;
        }

        /// <summary>
        /// Method AssertPickupRequest_ReisezeitfensterContraint
        /// </summary>
        /// <param name="old_state">A  VrpState</param>
        /// <param name="action">A  VrpAction</param>
        /// <returns>A  bool</retutns>
        private bool AssertPickupRequest_ReisezeitfensterContraint(VrpState old_state, VrpRequestAction action)
        {
           PickupRequest r = (PickupRequest)_requests[action.RequestIndex];

            // lastaction cannot be null as a delivering action must have had 
            // an pickup action in advance

            //get the actual destination from the request
            Vector2f pos_dest = r.ToUtm;
            //get the current pos
            Vector2f curr_pos = old_state.CurrVehiclePos[action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = old_state.CurrTime[action.VehicleIndex];

            //curr_time at pos (needs to be calculated)
            float airline_len_meters;
            DateTime estimated_time_at_dest = GetEstimatedTimeAtPos(curr_pos, pos_dest, curr_time, out airline_len_meters);


            // to check the contraint 2 ways are now possible
            // 1. more time conuming one, is that i consider the actual time 
            // the request was picked up, therefore i have to travel back through the states till
            // i get the pickup time of this request
            VrpState curr_state = old_state;
            while (!(curr_state.Action is VrpRequestAction) || ((VrpRequestAction)curr_state.Action).RequestIndex != action.RequestIndex)
            {
                curr_state = (VrpState)curr_state.PreviousState;
            }
            DateTime r_time_pickup = curr_state.CurrTime[action.VehicleIndex];
            TimeSpan estimated_ride_time = estimated_time_at_dest - r_time_pickup;

            //check contraint
            if (estimated_time_at_dest > r.LDT || estimated_ride_time > r.GetMRT(_planner))
                return false;
            else
                return true;
        }

        /// <summary>
        /// Method AssertPickupRequest_WartezeitfensterContraint
        /// </summary>
        /// <param name="old_state">A  VrpState</param>
        /// <param name="action">A  VrpAction</param>
        /// <returns>A  bool</retutns>
        private bool AssertPickupRequest_WartezeitfensterContraint(VrpState old_state, VrpRequestAction action)
        {
            //get current time
            PickupRequest r = (PickupRequest)_requests[action.RequestIndex];

            //get the actual destination from the request
            Vector2f pos_dest = r.FromUtm;
            //get the current pos
            Vector2f curr_pos = old_state.CurrVehiclePos[action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = old_state.CurrTime[action.VehicleIndex];

            //maybe the vehicle is now for the first time occupied
            //in that case we have to consider that the vehicle is still in the depot
            if (curr_pos.X == 0 && curr_pos.Y == 0)
            {
                curr_pos = _vehicles[action.VehicleIndex].DepotPositionUtm;
                curr_time = _start_time;
            }

            //curr_time at pos (needs to be calculated)
            float travel_distance;
            DateTime estimated_time_at_dest = GetEstimatedTimeAtPos(curr_pos, pos_dest, curr_time, out travel_distance);

            //check contraint
            if (estimated_time_at_dest > r.LPT)
                return false;
            else
                return true;
        }
        #endregion

        #region Vehicle Constraint Checking
        /// <summary>
        /// Überprüft ob alle vehicle constraints passen, sollte ein contraint verletzt werden,
        /// wird das false zurückgegeben.
        /// Es wird dabei das vehicle mit dem im action gespeicherten vehicle index evaluiert.
        /// </summary>
        /// <param name="action">A  VrpAction</param>
        protected virtual bool AssertBackwardVehicleConstraints(VrpState old_state, VrpRequestAction action)
        {
            Request r = _requests[action.RequestIndex];

            if (!AssertVehicleServicedArea(old_state, action))
                return false;

            if (action.IsPickup)
                return AssertVehiclePickup(old_state, action);
            else
                return AssertVehicleDelivery(old_state, action);
        }

        
        /// <summary>
        /// Method AssertVehicleServicedArea
        /// </summary>
        /// <returns>A  bool</retutns>
        protected bool AssertVehicleServicedArea(VrpState old_state, VrpRequestAction action)
        {
            if (_vehicles[action.VehicleIndex].ServicedArea == null)
                return true;

            return _cache_servicedarea_pointinpolycheck[action.VehicleIndex][action.RequestIndex];
        }
        /// <summary>
        /// Method AssertVehicleDelivery
        /// </summary>
        /// <param name="old_state">A  VrpState</param>
        /// <param name="action">A  VrpAction</param>
        /// <returns>A  bool</retutns>
        private bool AssertVehicleDelivery(VrpState old_state, VrpRequestAction action)
        {
            return true;
        }

        /// <summary>
        /// Method AssertVehiclePickup
        /// </summary>
        /// <param name="old_state">A  VrpState</param>
        /// <param name="action">A  VrpAction</param>
        /// <returns>A  bool</retutns>
        private bool AssertVehiclePickup(VrpState old_state, VrpRequestAction action)
        {

            //capacity constraint check
            if (_vehicles[action.VehicleIndex].capacity < old_state.VehicleUtilization[action.VehicleIndex] + 1)
                return false;
            else
                return true;
        }
        #endregion

        #region StopAction Contraint Checking
        private bool AssertBackwardConstraintsStopAction(VrpState old_state, VrpStopAction action)
        {

            //get the actual destination from the request
            Vector2f pos_dest = action.StopPosition;
            //get the current pos
            Vector2f curr_pos = old_state.CurrVehiclePos[action.VehicleIndex];
            //get time at current pos
            DateTime curr_time = old_state.CurrTime[action.VehicleIndex];

            //maybe the vehicle is now for the first time occupied
            //in that case we have to consider that the vehicle is still in the depot
            if (curr_pos.X == 0 && curr_pos.Y == 0)
            {
                curr_pos = _vehicles[action.VehicleIndex].DepotPositionUtm;
                curr_time = _start_time;
            }

            //curr_time at pos (needs to be calculated)
            float travel_distance;
            DateTime estimated_time_at_dest = GetEstimatedTimeAtPos(curr_pos, pos_dest, curr_time, out travel_distance);

            //check contraint
            if (estimated_time_at_dest > action.LST)
                return false;
            else
                return true;
        }
        #endregion

        #endregion

        #endregion

        #region Generel Properties
        /// <summary>
        /// gibt die anzahl der Actions zurück die durchgeführt werden müssen um einen Lösungsstate zu erreichen.
        /// handelt es sich um eine problem ohne info soll hier bzw. wird -1 zurückgegeben werden
        /// </summary>
        public override int CountActions
        {
            get
            {
                return _requests.Count * 2 + _custom_actions_count;
            }
        }

        public List<Vehicle> Vehicles
        {
            set
            {
                _vehicles = value;
            }

            get
            {
                return _vehicles;
            }
        }

        public List<Request> Requests
        {
            set
            {
                _requests = value;
            }

            get
            {
                return _requests;
            }
        }

        /// <summary>
        /// Setzt oder gibt zurück die Anzahl der Fahrzeuge die im Einsatz verwendet werden
        /// </summary>
        public int CountVehicles
        {
            set
            {
                _count_vehicles = value;
            }

            get
            {
                return _count_vehicles;
            }
        }

        /// <summary>
        /// setzt oder gibt zurück die anzahl der personen die eine Pickup and Delivery Anfrage gestartet haben
        /// </summary>
        public int CountRequestingPersons
        {
            set
            {
                _count_requesting_persons = value;
            }

            get
            {
                return _count_requesting_persons;
            }
        }

        public DateTime StartTime
        {
            set
            {
                _start_time = value;
            }

            get
            {
                return _start_time;
            }
        }
        /// <summary>
        /// wird diese variable auf true gesetzt bedeutet dies das sich die zeit (abholzeit) verändern kann/darf.
        /// dies gilt nur bzw. wird dann gemacht wenn create new state aufgerufen wird.
        /// sollte es hier zu einer ein anpassung des EPT kommen kann der vorgänger pickup theoretisch auch angepasst werden
        /// diese anpassung findet genau dann statt wenn dieses attrib auf true gesetzt wird.
        /// noch was: die timewindow contraction gilt nur für DELIVERY REQUESTS, denn genau dort tritt das problem auf.
        /// das problem vielleicht kurz beschrieben. das löschen von actions einer bestehenden lösung ist grundsätzlich 
        /// kein problem, damit meine ich das das löschen ohne check der constraints gemacht werden kann. ausser bei delivery 
        /// requests kann es zu problemen kommen wenn man actions löscht. bsp.: die startzeit verschiebt sich durch wegfall 
        /// von vorgängern weiter nach hinten bsp.: 15:30 auf 15:20, das heisst ein delivery request kann bereits um 15:20
        /// abgeholt werden. die actions zwischen pickup und delivery bleiben gleich, jedoch wird das EPT der actions dazwischen 
        /// übernommen weil zwar jetzt früher die anderen actions bedient werden können, jedoch haben diese ein ept das eingehalten werden
        /// muss, die wartezeit wird angepasst. in summe kann es nun passieren das die MRT überschritten wird vom ersten request
        /// da dieser früher abgeholt wird, jedoch trotzdem gleich lang reisen muss. die lösung zu diesem problem liegt im
        /// 1. übernahme der abholzeit der alten state struktur
        /// oder
        /// 2. time window contraction auf true setzen, --> verändere nach dem anpassen eines  ept im nachfolger auch die vorgänger abholzeit
        /// </summary>
        public bool WithTimeContraction
        {
            set
            {
                _with_time_contraction = value;
            }

            get
            {
                return _with_time_contraction;
            }
        }

        public Routeplanner Planner
        {
            set
            {
                _planner = value;
            }

            get
            {
                return _planner;
            }
        }
        #endregion

        #region Attribs
        protected List<Vehicle> _vehicles;
        protected List<Request> _requests;
        protected int _count_vehicles;
        protected int _count_requesting_persons;
        protected DateTime _start_time;
        protected List<List<bool>> _cache_servicedarea_pointinpolycheck;
        protected Routeplanner _planner;
        /// <summary>
        /// wird diese variable auf true gesetzt bedeutet dies das sich die zeitfenster verändern können dürfen.
        /// dies gilt nur bzw. wird dann gemacht wenn create new state aufgerufen wird.
        /// sollte es hier zu einer ein anpassung des EPT kommen kann der vorgänger pickup theoretisch auch angepasst werden
        /// diese anpassung findet genau dann statt wenn dieses attrib auf true gesetzt wird.
        /// noch was: die timewindow contraction gilt nur für DELIVERY REQUESTS, denn genau dort tritt das problem auf.
        /// das problem vielleicht kurz beschrieben. das löschen von actions einer bestehenden lösung ist grundsätzlich 
        /// kein problem, damit meine ich das das löschen ohne check der constraints gemacht werden kann. ausser bei delivery 
        /// requests kann es zu problemen kommen wenn man actions löscht. bsp.: die startzeit verschiebt sich durch wegfall 
        /// von vorgängern weiter nach hinten bsp.: 15:30 auf 15:20, das heisst ein delivery request kann bereits um 15:20
        /// abgeholt werden. die actions zwischen pickup und delivery bleiben gleich, jedoch wird das EPT der actions dazwischen 
        /// übernommen weil zwar jetzt früher die anderen actions bedient werden können, jedoch haben diese ein ept das eingehalten werden
        /// muss, die wartezeit wird angepasst. in summe kann es nun passieren das die MRT überschritten wird vom ersten request
        /// da dieser früher abgeholt wird, jedoch trotzdem gleich lang reisen muss. die lösung zu diesem problem liegt im
        /// 1. übernahme der abholzeit der alten state struktur
        /// oder
        /// 2. time window contraction auf true setzen, --> verändere nach dem anpassen eines  ept im nachfolger auch die vorgänger abholzeit
        /// </summary>
        protected bool _with_time_contraction = false;


        #endregion

    }

}
