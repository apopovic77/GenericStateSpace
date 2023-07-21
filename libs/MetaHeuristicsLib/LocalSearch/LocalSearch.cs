using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using MathLib;
using System.Collections;


namespace Logicx.Optimization.GenericStateSpace.LocalSearch
{
    /// <summary>
    /// this class searches one time through a statespace and returns the thereby best solution found.
    /// this mechanism is aware of different forms of statespaces.
    /// Backtracking is provided. This means if the path followed does not lead to a solution,
    /// backtracking will be performend
    /// </summary>
    public class LocalSearch
    {
        public LocalSearch(StateSpace statespace,State last_best_state)
        {
            _statespace = statespace;
            _last_best_state = last_best_state;
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

        public State SolutionState
        {
            set
            {
                _last_best_state = value;
            }

            get
            {
                return _last_best_state;
            }
        }

        /// <summary>
        /// call this method to start the local search algorithm
        /// this method will try via swapping actions to improve the solution
        /// if this is successfull true will be returned
        /// </summary>
        /// <returns>if there is a better solution found through the use of local search, true will be returned</returns>
        public bool Run()
        {
            bool sol_improved = false;
            while (true) {
                if (RunOneIteration())
                    sol_improved = true;
                else
                    return sol_improved;
            }
        }

        private bool RunOneIteration() {
            //create helper vars for the iteration 
            State curr_state = _last_best_state;
            //store the last best improved state in here
            State local_improved_state = _last_best_state;
            //create a list for the action stack
            List<Action> action_list = new List<Action>();

            //start swap algorithm
            while (curr_state.PreviousState.PreviousState != null)
            {
                //save org action in list
                action_list.Add(curr_state.Action);

                State swapped_state = Swap(curr_state, 0, 1, action_list);
                if (swapped_state != null)
                {
                    if (swapped_state.CurrentTargetValue < local_improved_state.CurrentTargetValue)
                        local_improved_state = swapped_state;
                }

                curr_state = curr_state.PreviousState;
            }

            //ajdust last best state if local improved version is better
            if (local_improved_state.CurrentTargetValue < _last_best_state.CurrentTargetValue)
            {
                _last_best_state = local_improved_state;
                return true;
            }
            else
            {
                return false;
            }        
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="curr_state_clone"></param>
        /// <param name="prev_state_clone"></param>
        /// <param name="prev_prev_state_clone"></param>
        /// <param name="action_list"></param>
        /// <returns></returns>
        private State Swap(State curr_state,int action_index_1,int action_index_2, List<Action> action_list)
        {

            if (curr_state.PreviousState.PreviousState == null) throw new System.Exception("There is no root state, cannot execute swap");

            //set the previous previous state as the state
            //where we start the swap algorithm
            State swap_root_state = curr_state.PreviousState.PreviousState;
            
            //get the actions that are to be swapped
            Action swap_action_1 = curr_state.Action;
            Action swap_action_2 = curr_state.PreviousState.Action;

            //do first swap
            swap_root_state = _statespace.NextState(swap_root_state, swap_action_1);
            if (swap_root_state == null)
                return null;

            //do second swap
            swap_root_state = _statespace.NextState(swap_root_state, swap_action_2);
            if (swap_root_state == null)
                return null;

            //check the rest of the following actions
            //if the constraints hold we found a new swapped solution
            for (int j = action_list.Count - 2; j >= 0; j--)
            {
                swap_root_state = _statespace.NextState(swap_root_state, action_list[j]);
                if (swap_root_state == null)
                    return null;
            }
            return swap_root_state;
        }


        #region attribs
        protected StateSpace _statespace;
        protected State _last_best_state;
        #endregion
    }
}
