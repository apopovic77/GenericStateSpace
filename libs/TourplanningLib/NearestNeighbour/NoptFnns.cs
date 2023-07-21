using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP;
using MathLib;
using Logicx.Utilities;

namespace Logicx.Optimization.GenericStateSpace.NearestNeighbour
{
    /// <summary>
    /// this class searches one time through a statespace and returns the thereby best solution found.
    /// this mechanism is aware of different forms of statespaces.
    /// Backtracking is provided. This means if the path followed does not lead to a solution,
    /// backtracking will be performend
    /// </summary>
    public class NoptFnns
    {
        public NoptFnns(StateSpace statespace)
        {
            _statespace = statespace;
        }

        public List<State> SubSolutionStates
        {
            set
            {
                _sub_solution_states = value;
            }

            get
            {
                return _sub_solution_states;
            }
        }

        public State SolutionState
        {

            get
            {

                if (_solution_state != null)
                    return _solution_state;

                List<Request> requests = ((VrpStateSpace)_statespace).Requests;

                //set starting state
                _statespace.InitializeFirstState();
                _solution_state = _statespace.InitialState;

                //create the solution state
                for (int i = 0; i < _sub_solution_states.Count; i++)
                {

                    //build action list for vehicle i
                    List<Action> action_list = new List<Action>();
                    State final_state = _sub_solution_states[i];
                    while (final_state.PreviousState != null)
                    {
                        action_list.Add(final_state.Action);
                        final_state = final_state.PreviousState;
                    }


                    ((VrpStateSpace)_statespace).WithTimeContraction = true;


                    //add actions to final solution list
                    for (int action_index = action_list.Count - 1; action_index >= 0; action_index--)
                    {
                        VrpRequestAction action = (VrpRequestAction)action_list[action_index];
                        VrpRequestAction org_action = new VrpRequestAction(i, requests.IndexOf(_list_served_requests[i][action.RequestIndex]), action.IsPickup);
                        int index_org_ref_action = requests.Count * 2 * org_action.VehicleIndex + org_action.RequestIndex * 2 + ((org_action.IsPickup) ? 0 : 1);
                        Action org_ref_action = _statespace.AllPossibleActions[index_org_ref_action];

                        _solution_state = _statespace.NextState(_solution_state, org_ref_action);
                    }

                    ((VrpStateSpace)_statespace).WithTimeContraction = false;
                }

                return _solution_state;
            }
        }


        public StateSpace StateSpace
        {
            set
            {
                _statespace = value;
            }

            get
            {
                return _statespace;
            }
        }

        public IDebugWriter DebugWriter
        {
            set
            {
                _debugwriter = value;
            }

            get
            {
                return _debugwriter;
            }
        }
		
		public int LookAhead
		{
			set {
                if (_look_ahead < 2)
                    throw new Exception("Use the normal FNNS if you want look ahead = 1");
				_look_ahead = value;
			}
			
			get {
				return _look_ahead;
			}
		}
		
		public List<List<Request>> SubServedRequests
		{
			set {
				_list_served_requests = value;
			}
			
			get {
				return _list_served_requests;
			}
		}

        public void Run()
        {

                _list_served_requests = new List<List<Request>>();

                _sub_solution_states = new List<State>();
                VrpStateSpace vrpstatespace = (VrpStateSpace)_statespace;
                
                List<Request> requests = new List<Request>(vrpstatespace.Requests);
                WriteDebug("ALL R COUNT " + requests.Count);

                for (int k = 0; k < vrpstatespace.Vehicles.Count; k++)
                {

                    if (requests.Count == 0)
                        break;

                    VrpStateSpace statespace_k = new VrpStateSpace(vrpstatespace.Vehicles.GetRange(k, 1), requests, vrpstatespace.StartTime,vrpstatespace.Planner);
                    int beaktracking_count = 0;

                    //save the state with the highest depth value
                    //it must not be that the state the at the has the highest depth value
                    State highest_depth_state = null;


                    State curr_state = null;
                    State next_state = null;
                    do
                    {

                        next_state = GetNoptMinCostState(curr_state, statespace_k, _look_ahead, 0);

                        #region backtracking
                        //check if backtracking needed
                        if (next_state == null)
                        {

                            #region saving highest depth state
                            if (highest_depth_state == null)
                                highest_depth_state = curr_state;
                            else if (highest_depth_state.DepthState < curr_state.DepthState)
                                highest_depth_state = curr_state;
                            #endregion

                            #region quit search save result
                            beaktracking_count++;
                            if (beaktracking_count > 1000 * (k + 1) || curr_state.PreviousState == null)
                            {
                                //use the state with the highest depth state
                                curr_state = highest_depth_state;

                                //TODO SAVE ALL PICKUPS WITHOUT DELIVERY AND TRY TO ADD IN SOME OTHER WAY
                                //PICKUPS THAT CANNOT BE DELIVERED SHOULD THAN REALLY BE DELIVERED WITH ANOTHER CAR

                                while (((VrpRequestAction)curr_state.Action).IsPickup)
                                {
                                    curr_state = curr_state.PreviousState;
                                }
                                List<int> delivering_requests = new List<int>();

                                State final_state = curr_state;

                                State cur_state = null;
                                while (curr_state.Action != null)
                                {
                                    VrpRequestAction action = (VrpRequestAction)curr_state.Action;
                                    int del_index = delivering_requests.IndexOf(action.RequestIndex);
                                    if (action.IsPickup && del_index < 0)
                                    {
                                        cur_state.PreviousState = curr_state.PreviousState;
                                        curr_state = cur_state.PreviousState;
                                        curr_state.LastQueriedNextState = cur_state;
                                        continue;
                                    }
                                    else if (action.IsPickup)
                                    {
                                        delivering_requests.RemoveAt(del_index);
                                    }
                                    else
                                    {
                                        delivering_requests.Add(action.RequestIndex);
                                    }
                                    cur_state = curr_state;
                                    curr_state = cur_state.PreviousState;
                                    curr_state.LastQueriedNextState = cur_state;
                                }

                                curr_state = final_state;
                                break;
                            }
                            #endregion

                            //we are stuck in an edge not leading to a solution
                            //remove this state from the parent state
                            //we cannot find a solution following this path
                            curr_state.PreviousState.Remove(curr_state);


                            //for (int i = 0; i < _look_ahead-1; i++)
                            //{
                            //    curr_state = curr_state.PreviousState;
                            //}


                            //indicate that a state has been deleted
                            _statespace.StatesExistent--;
                            //start again with the parent
                            //but this time without considering this state as a possible path
                            curr_state = curr_state.PreviousState;
                            continue;
                        }
                        #endregion

                        curr_state = next_state;
                    } while (curr_state == null || curr_state.DepthState < statespace_k.CountActions);

                    //save result
                    _list_served_requests.Add(new List<Request>(statespace_k.Requests));
                    _sub_solution_states.Add(curr_state);

                    //Console.WriteLine(statespace_k.GetServedRequests((VrpState)prev_state, 0).Count);
                    //List<List<VrpStateSpace.VrpWorkItem>> vehicle_workitem_list_nns = statespace_k.BuildSeperateActionsLists((VrpState)prev_state);
                    //PrintSolution(statespace_k.Requests, vehicle_workitem_list_nns, prev_state.CurrentTargetValue);

                    //delete from org requests list
                    List<Request> served_requests = statespace_k.GetServedRequests((VrpState)curr_state, 0);
                    for (int i = 0; i < served_requests.Count; i++)
                        requests.Remove(served_requests[i]);
                }

        }

        private State GetNoptMinCostState(State curr_state, StateSpace statespace, int nopt_depth, int curr_depth)
        {
            int real_state_depth = (curr_state != null) ? curr_state.DepthState : 0;
            if (real_state_depth + nopt_depth > statespace.CountActions)
            {
                nopt_depth = statespace.CountActions - real_state_depth;
            }

            State min_cost_next_state = null;
            GetInDepthState(curr_state, statespace, nopt_depth, curr_depth + 1, ref min_cost_next_state);

            //if (min_cost_next_state == null)
            //    return null;
            ////for (int i = 0; i < nopt_depth-1; i++)
            ////{
            ////    min_cost_next_state = min_cost_next_state.PreviousState;
            ////}
            return min_cost_next_state;
        }

        private void GetInDepthState(State curr_state, StateSpace statespace, int nopt_depth, int curr_depth,ref State curr_in_depth_min_cost_state)
        {

            List<State> next_states = statespace.NextStates(curr_state);
            
            for (int i = 0; i < next_states.Count; i++)
            {
                if (curr_depth < nopt_depth)
                {
                    GetInDepthState(next_states[i], statespace, nopt_depth, curr_depth + 1, ref  curr_in_depth_min_cost_state);
                }
                else
                {
                    if (curr_in_depth_min_cost_state == null || next_states[i].CurrentTargetValue < curr_in_depth_min_cost_state.CurrentTargetValue)
                        curr_in_depth_min_cost_state = next_states[i];
                }
            }
        }


        #region Debug stuff
        public void WriteDebug(string msg)
        {
            if (_debugwriter != null)
                _debugwriter.WriteInfo(msg);
        }
        #endregion

        //public static void PrintSolution(List<Request> lr, List<List<VrpStateSpace.VrpWorkItem>> v_workitem_list, float distance_traveled)
        //{

        //    Console.WriteLine("Minimum Distance overall " + distance_traveled);
        //    for (int i = 0; i < v_workitem_list.Count; i++)
        //    {
        //        Console.Write("V" + i + ": ");
        //        for (int j = 0; j < v_workitem_list[i].Count; j++)
        //        {

        //            VrpStateSpace.VrpWorkItem workitem = (VrpStateSpace.VrpWorkItem)v_workitem_list[i][j];
        //            string name;
        //            if (workitem.Action.IsPickup)
        //                name = lr[workitem.Action.RequestIndex].from_name;
        //            else
        //                name = lr[workitem.Action.RequestIndex].to_name;
        //            Console.Write("{R" + workitem.Action.RequestIndex + ((workitem.Action.IsPickup) ? "P" : "D") + " " + name + ":" + workitem.TimeArrival.ToShortTimeString() + "}");
        //            if (j < v_workitem_list[i].Count - 1)
        //            {
        //                Console.Write(",");
        //            }
        //        }
        //        Console.WriteLine();
        //    }
        //    Console.WriteLine();

        //}
        protected StateSpace _statespace;
        protected List<State> _sub_solution_states;
        protected List<List<Request>> _list_served_requests;
        protected State _solution_state;
        protected IDebugWriter _debugwriter;
        protected int _look_ahead = 2;
    }
}
