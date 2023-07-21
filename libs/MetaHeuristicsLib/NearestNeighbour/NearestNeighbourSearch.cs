using System.Collections.Generic;
using Logicx.Optimization.GenericStateSpace;
using Logicx.Utilities;

namespace Logicx.Optimization.MetaHeuristics.NearestNeighbour
{
    /// <summary>
    /// this class searches one time through a statespace and returns the thereby best solution found.
    /// this mechanism is aware of different forms of statespaces.
    /// Backtracking is provided. This means if the path followed does not lead to a solution,
    /// backtracking will be performend
    /// </summary>
    public class NearestNeighbourSearch
    {
        public NearestNeighbourSearch(StateSpace statespace) 
        {
            _statespace = statespace;
        }
		
		public State SolutionState
		{
			set {
				_final_solution_state = value;
			}
			
			get {
				return _final_solution_state;
			}
		}

	
		public StateSpace StateSpace
		{
			set {
				_statespace = value;
			}
			
			get {
				return _statespace;
			}
		}
		
		public IDebugWriter DebugWriter
		{
			set {
				_debugwriter = value;
			}
			
			get {
				return _debugwriter;
			}
		}

        public void Run() {

            State prev_state = null;
            do
            {
                //fetch next states
                List<State> next_states = _statespace.NextStates(prev_state);

                //check if backtracking needed
                if (next_states.Count == 0) {
                    if (prev_state == null || prev_state.PreviousState == null)
                        return; //no solution given
                    //we are stuck in an edge not leading to a solution
                    //remove this state from the parent state
                    //we cannot find a solution following this path
                    prev_state.PreviousState.Remove(prev_state);
                    //indicate that a state has been deleted
                    _statespace.StatesExistent--;
                    //start again with the parent
                    //but this time without considering this state as a possible path
                    prev_state = prev_state.PreviousState;
                    continue;
                }
                //create vars needed to evaluate next best action leading to the best next state
                int index_min_cost_next_state = -1;
                float min_cost = float.MaxValue;

                //go through all edges and evaluate best next state
                float prev_target_value = (prev_state==null)?0f:prev_state.CurrentTargetValue;
                for (int i = 0; i < next_states.Count; i++)
                {
                    State state = next_states[i];
                    float cost_action = state.CurrentTargetValue - prev_target_value;
                    if( cost_action < min_cost)
                    {
                        index_min_cost_next_state = i;
                        min_cost = cost_action;
                    }
                }

                prev_state = next_states[index_min_cost_next_state];
            } while (prev_state.DepthState < _statespace.CountActions);

            _final_solution_state = prev_state;
        }

        protected StateSpace _statespace;
        protected State _final_solution_state;
        protected IDebugWriter _debugwriter;
    }
}
