using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;

namespace Logicx.Optimization.GenericStateSpace.NearestNeighbour
{
    public class NoptNns
    {
        public NoptNns(StateSpace statespace)
        {
            _statespace = statespace;
        }
		
		public State SolutionState
		{
			set {
				_solstate = value;
			}
			
			get {
				return _solstate;
			}
		}
		
		public int LookAhead
		{
			set {
				_look_ahead = value;
			}
			
			get {
				return _look_ahead;
			}
		}


        public void Run()
        {
            //save the state with the highest depth value
            //it must not be that the state the at the has the highest depth value
            State highest_depth_state = null;
            State curr_state = null;
            State next_state = null;
            do
            {

                next_state = GetNoptMinCostState(curr_state, _statespace, _look_ahead, 0);

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
                    if (curr_state.PreviousState == null)
                    {
                        throw new Exception("no solution can be found");
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

            } while (curr_state == null || curr_state.DepthState < _statespace.CountActions);


            _solstate = curr_state;
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

            return min_cost_next_state;
        }


        private void GetInDepthState(State curr_state, StateSpace statespace, int nopt_depth, int curr_depth, ref State curr_in_depth_min_cost_state)
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

        protected State _solstate;
        protected StateSpace _statespace;
        protected int _look_ahead = 2;
    }
}
