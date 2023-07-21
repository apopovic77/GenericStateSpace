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
    public class FastNearestNeighbourSearch2
    {
        public FastNearestNeighbourSearch2(StateSpace statespace)
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

                //set starting state
                _statespace.InitializeFirstState();
                _solution_state = _statespace.InitialState;

                _solution_state = _list_final_solutions[0];

                ////create the solution state
                //for (int i = 0; i < _list_final_solutions.Count; i++)
                //{

                //    if (_list_final_solutions[i] == null)
                //        continue;

                //    //XBUG: the problem is that we still did no solve the bug problem with ReconstructContractedState
                //    //sometimes this method failes, in most of the time it works, iam leaving it now like that
                //    //until we find time to correct the bug in [ReconstructContractedState]
                //    //it is easy to know when the method fails: it returns null
                //    VrpState contracted_solstate = ((VrpStateSpace) _statespace).ReconstructContractedState(
                //        (VrpState) _solution_state, (VrpState) _list_final_solutions[i], _list_served_requests[i], i);
                //    if (contracted_solstate == null)
                //        _solution_state = ((VrpStateSpace) _statespace).ReconstructState((VrpState) _solution_state,
                //            (VrpState) _list_final_solutions[i], _list_served_requests[i], i);
                //    else
                //        _solution_state = contracted_solstate;
                //}

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

        public void Run()
        {
            _list_final_solutions = new List<State>();

            //the last state known
            State curr_state = null;
            do
            {
                //fetch next states
                List<State> next_states = _statespace.NextStates(curr_state);

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


            } while (curr_state.DepthState < _statespace.CountActions);

           _list_final_solutions.Add(curr_state);
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
        protected State _solution_state;
        protected IDebugWriter _debugwriter;
        protected bool _with_second_chance = false;
        protected int _backtracking_base_count = 1000;
        protected bool _with_insertion_of_discarded_requests = false;

        #endregion
    }
}

