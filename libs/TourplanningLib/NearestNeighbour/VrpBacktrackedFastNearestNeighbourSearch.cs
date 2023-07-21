using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP;
using MathLib;

using Logicx.Utilities;
using Action = Logicx.Optimization.GenericStateSpace.Action;


namespace Logicx.Optimization.Tourplanning.NearestNeighbour
{
    /// <summary>
    /// this class searches one time through a statespace and returns the thereby best solution found.
    /// this mechanism is aware of different forms of statespaces.
    /// Backtracking is provided. This means if the path followed does not lead to a solution,
    /// backtracking will be performend
    /// </summary>
    public class VrpBacktrackedFastNearestNeighbourSearch
    {
        public VrpBacktrackedFastNearestNeighbourSearch(StateSpace statespace)
        {
            _statespace = statespace;
        }

        public List<State> ListSolutionStates
        {
            set { _list_final_solutions = value; }

            get { return _list_final_solutions; }
        }


        /// <summary>
        /// the fast nearest neighbour search splits the statespace into substate spaces
        /// here you can query an again fused state of all substates that have been gained
        /// </summary>
        public State SolutionState
        {

            get
            {

                // return immediatelly the statespace if it has been fused already
                if (_solution_state != null)
                    return _solution_state;


                //get the full requests list and save it in a new var
                List<Request> requests = ((VrpStateSpace) _statespace).Requests;

                //set starting state
                _statespace.InitializeFirstState();
                _solution_state = _statespace.InitialState;

                //create the solution state
                for (int i = 0; i < _list_final_solutions.Count; i++)
                {

                    if (_list_final_solutions[i] == null)
                        continue;

                    //XBUG: the problem is that we still did no solve the bug problem with ReconstructContractedState
                    //sometimes this method failes, in most of the time it works, iam leaving it now like that
                    //until we find time to correct the bug in [ReconstructContractedState]
                    //it is easy to know when the method fails: it returns null
                    VrpState contracted_solstate = ((VrpStateSpace) _statespace).ReconstructContractedState(
                        (VrpState) _solution_state, (VrpState) _list_final_solutions[i], _list_served_requests[i], i);
                    if (contracted_solstate == null)
                        _solution_state = ((VrpStateSpace) _statespace).ReconstructState((VrpState) _solution_state,
                            (VrpState) _list_final_solutions[i], _list_served_requests[i], i);
                    else
                        _solution_state = contracted_solstate;
                }

                return _solution_state;
            }
        }


        public StateSpace StateSpace
        {
            set { _statespace = value; }

            get { return _statespace; }
        }

        public IDebugWriter DebugWriter
        {
            set { _debugwriter = value; }

            get { return _debugwriter; }
        }


        public int BacktrackingBaseCount
        {
            set { _backtracking_base_count = value; }

            get { return _backtracking_base_count; }
        }

        public bool WithSecondChance
        {
            set { _with_second_chance = value; }

            get { return _with_second_chance; }
        }

        public bool WithInsertionOfDiscardedRequests
        {
            set { _with_insertion_of_discarded_requests = value; }

            get { return _with_insertion_of_discarded_requests; }
        }

        public void Run()
        {
            //init local vars needed for the algorithm
            _list_served_requests = new List<List<Request>>();
            _list_final_solutions = new List<State>();
            VrpStateSpace vrpstatespace = (VrpStateSpace) _statespace;
            List<Request> requests = new List<Request>(vrpstatespace.Requests);

            // run the alg sukkzesive for each vehicle
            for (int k = 0; k < vrpstatespace.Vehicles.Count; k++)
            {

                //XBUG: SEED ALG MISSING
                //it is needed to check which vehicle to be used next,
                //here the next vehicle independent to its position is taken
                //several algs are possible here:
                //1. the vehicle most near to the next request is taken
                //2. the vehicle is lead to the request with the highest
                //   time congruency value is taken


                //create a sub statespace vor the k th vehicle
                //here we always have one vehicle on the statespace
                //VrpStateSpace statespace_k = new VrpAdvStateSpace(vrpstatespace.Vehicles.GetRange(k, 1), requests, vrpstatespace.StartTime,vrpstatespace.Planner);

                ////XBUG: THINK ABOUT A SUITING OPTION FOR CACHING

                ////statespace_k.CachingStrategy = StateSpace.Cachingstrategy.No_Caching;
                ////initialize immediatelly the first statespace
                //statespace_k.InitializeFirstState();


                VrpStateSpace statespace_k = ((VrpStateSpace) _statespace).SplitSpace(k, requests);


                //if all requests have been served we dont need anymore 
                //if (requests.Count == 0)
                //    break;
                if (statespace_k.AllPossibleActions.Count == 0)
                {
                    _list_served_requests.Add(null);
                    _list_final_solutions.Add(null);
                    continue;
                }



                //set local vars need for the alg which a separate for each vehicle

                //we need this var for the next chance alg
                //if no backtracking is allowed anymore, the alg quits to search for a better solution for the k th vehicle
                //in that case we quit and start with the next vehicle
                //we use this flag to inidicate if we break the alg execution for the first time
                //if we set the withnextchance flag to true we allow to run the alg again the k th vehicle
                //but this time the alg starts with the known best solution from before
                bool first_break = true;
                //here we save how many times we did the backtracking
                int backtracking_count = 0;
                //it might be possible that because of backtracking the next best state does not serve as much requests as
                //an previous found solution state
                //here we save the state with the highest number of passengers served
                State highest_delivered_request_state = null;
                int highest_delivered_requests_count = 0;
                //this is a local runtime var that gives advice about the count of served request in the current sub state solution
                int requests_delivered = 0;

                //the last state known
                State curr_state = null;
                do
                {
                    //fetch next states
                    List<State> next_states = statespace_k.NextStates(curr_state);



                    #region backtracking

                    //check if backtracking needed
                    //this is the case when we cannot find a following state
                    if (next_states.Count == 0)
                    {
                        //here we are backtracking from the current reached state
                        //as we know that this must not mean that we will find after the backtracking a
                        //better state
                        if (highest_delivered_request_state == null)
                        {
                            highest_delivered_request_state = curr_state;
                            highest_delivered_requests_count = requests_delivered;
                        }
                        else if (highest_delivered_requests_count < requests_delivered)
                        {
                            highest_delivered_request_state = curr_state;
                            highest_delivered_requests_count = requests_delivered;
                        }



                        backtracking_count++;
                        if (backtracking_count > BacktrackingBaseCount * (k + 1) || curr_state == null ||
                            curr_state.PreviousState == null)
                        {
                            //use highest req delivered state
                            curr_state = highest_delivered_request_state;


                            if (curr_state != null)
                            {
                                List<int> discarded_requests = null;
                                if (_with_insertion_of_discarded_requests)
                                    discarded_requests = new List<int>();


                                //lösche die ersten pickups weg für die es keine delivery gibt
                                while (true)
                                {
                                    if (!(curr_state.Action is VrpRequestAction))
                                        break;

                                    if (!((VrpRequestAction) curr_state.Action).IsPickup)
                                        break;

                                    if (_with_insertion_of_discarded_requests)
                                        discarded_requests.Add(((VrpRequestAction) curr_state.Action).RequestIndex);
                                    curr_state = curr_state.PreviousState;
                                }


                                List<Action> actions = new List<Action>();

                                //lösche alle pickups ohne delivery weg
                                List<int> delivering_requests = new List<int>();
                                State final_state = curr_state;
                                State curr_state_checking_for_discarding_requests = null;
                                while (curr_state.Action != null)
                                {
                                    if (curr_state.Action is VrpRequestAction)
                                    {
                                        VrpRequestAction action = (VrpRequestAction) curr_state.Action;
                                        int del_index = delivering_requests.IndexOf(action.RequestIndex);
                                        if (action.IsPickup && del_index < 0)
                                        {

                                            if (_with_insertion_of_discarded_requests)
                                                discarded_requests.Add(((VrpRequestAction) action).RequestIndex);

                                            curr_state_checking_for_discarding_requests.PreviousState =
                                                curr_state.PreviousState;
                                            curr_state = curr_state_checking_for_discarding_requests.PreviousState;
                                            curr_state.LastQueriedNextState =
                                                curr_state_checking_for_discarding_requests;
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
                                    }


                                    curr_state_checking_for_discarding_requests = curr_state;
                                    curr_state = curr_state_checking_for_discarding_requests.PreviousState;
                                    curr_state.LastQueriedNextState = curr_state_checking_for_discarding_requests;

                                    //save all actions
                                    actions.Add(curr_state.LastQueriedNextState.Action);
                                }

                                State new_state_built = null;

                                if (_with_insertion_of_discarded_requests || _with_second_chance)
                                {
                                    try
                                    {
                                        //with time contraction
                                        statespace_k.WithTimeContraction = true;

                                        new_state_built = statespace_k.InitialState;
                                        new_state_built.LastQueriedNextState = null;
                                        new_state_built.NextStates = null;
                                        for (int i = actions.Count - 1; i >= 0; i--)
                                        {
                                            new_state_built = statespace_k.NextState(new_state_built, actions[i]);
                                            if (new_state_built == null)
                                                break;
                                            new_state_built.NextStates = null;
                                        }

                                    }
                                    finally
                                    {
                                        statespace_k.WithTimeContraction = false;
                                    }
                                }

                                if (_with_insertion_of_discarded_requests && new_state_built != null)
                                {
                                    final_state = TryInsertDiscardedRequests(discarded_requests, new_state_built,
                                        statespace_k);
                                }

                                curr_state = final_state;


                                if (_with_second_chance && first_break && new_state_built != null)
                                {

                                    first_break = false;
                                    backtracking_count = 0;

                                    highest_delivered_request_state = new_state_built;
                                    highest_delivered_requests_count = new_state_built.DepthState / 2;

                                    requests_delivered = highest_delivered_requests_count;

                                    curr_state = new_state_built;
                                    continue;
                                }
                            }

                            break;
                        }


                        //we are stuck in an edge not leading to a solution
                        //remove this state from the parent state
                        //we cannot find a solution following this path
                        curr_state.PreviousState.Remove(curr_state);
                        //indicate that a state has been deleted
                        _statespace.StatesExistent--;
                        //update request delivered count
                        if (curr_state.Action is VrpRequestAction && !((VrpRequestAction) curr_state.Action).IsPickup)
                            requests_delivered--;
                        //start again with the parent
                        //but this time without considering this state as a possible path
                        curr_state = curr_state.PreviousState;
                        continue;
                    }

                    #endregion

                    #region eval next best node

                    //create vars needed to evaluate next best action leading to the best next state
                    int index_min_cost_next_state = -1;
                    float min_cost = float.MaxValue;

                    //go through all edges and evaluate best next state
                    float prev_target_value = (curr_state == null) ? 0f : curr_state.CurrentTargetValue;
                    for (int i = 0; i < next_states.Count; i++)
                    {
                        State state = next_states[i];
                        float cost_action = state.CurrentTargetValue - prev_target_value;
                        if (cost_action < min_cost)
                        {
                            index_min_cost_next_state = i;
                            min_cost = cost_action;
                        }
                    }

                    #endregion

                    //set the next best state as the new current state
                    curr_state = next_states[index_min_cost_next_state];

                    //we are saving here how many requests have been deliverd
                    //by the current car, to achieve that we check if the last aciton
                    //was a delivery, if so increase the counter
                    if (curr_state.Action is VrpRequestAction && !((VrpRequestAction) curr_state.Action).IsPickup)
                    {
                        requests_delivered++;
                    }


                } while (curr_state.DepthState < statespace_k.CountActions);

                //save result
                _list_served_requests.Add(new List<Request>(statespace_k.Requests));
                _list_final_solutions.Add(curr_state);

                //delete from org requests list
                List<Request> served_requests = statespace_k.GetServedRequests((VrpState) curr_state, 0);
                for (int i = 0; i < served_requests.Count; i++)
                    requests.Remove(served_requests[i]);
            }
        }

        private State TryInsertDiscardedRequests(List<int> discarded_requests, State final_state,
            VrpStateSpace statespace)
        {
            if (discarded_requests.Count == 0)
                return final_state;

            //XBUG - LOW: hier die request liste nach dezentralisierungs index sortieren und dann der reihe nach einfügen

            //save the final state
            State new_final_state = final_state;
            for (int i = 0; i < discarded_requests.Count; i++)
            {
                int request_index = discarded_requests[i];
                final_state = statespace.InsertRequestOptimal(new_final_state, request_index);
                if (final_state != null)
                    new_final_state = final_state;
            }

            return new_final_state;
        }

        #region Debug stuff

        public void WriteDebug(string msg)
        {
            if (_debugwriter != null)
                _debugwriter.WriteInfo(msg);
        }

        #endregion

        #region Attributes

        protected StateSpace _statespace;
        protected List<State> _list_final_solutions;
        protected List<List<Request>> _list_served_requests;
        protected State _solution_state;
        protected IDebugWriter _debugwriter;
        protected bool _with_second_chance = false;
        protected int _backtracking_base_count = 1000;
        protected bool _with_insertion_of_discarded_requests = false;

        #endregion
    }
}

