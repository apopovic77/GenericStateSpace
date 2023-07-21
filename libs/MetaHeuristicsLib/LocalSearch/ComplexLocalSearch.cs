using Logicx.Optimization.GenericStateSpace;

namespace Logicx.Optimization.GenericStateSpace.LocalSearch
{
    /// <summary>
    /// this class searches one time through a statespace and returns the thereby best solution found.
    /// this mechanism is aware of different forms of statespaces.
    /// Backtracking is provided. This means if the path followed does not lead to a solution,
    /// backtracking will be performend
    /// </summary>
    public class ComplexLocalSearch
    {
        public ComplexLocalSearch(StateSpace statespace, State last_best_state)
        {
            _statespace = statespace;
            _last_best_state = last_best_state;
            //_swap_difference = swap_difference;
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

        public void Run()
        {
            State temp_best_state = _last_best_state;
            State temp_best_state_2 = _last_best_state;

            int swap_difference_limit = _last_best_state.DepthState - 1;
            for (int i = 1; i <= swap_difference_limit; i++)
            {                
                NoptLocalSearch n_opt_local_search = new NoptLocalSearch(_statespace, _last_best_state, i);
                if (n_opt_local_search.Run())
                    temp_best_state = n_opt_local_search.SolutionState;
                if (temp_best_state.CurrentTargetValue <= temp_best_state_2.CurrentTargetValue)
                {
                    temp_best_state_2 = temp_best_state;
                    //_swap_difference = i;
                }                
            }
            _last_best_state = temp_best_state_2;
            NoptLocalSearch n_opt_local_search2 = new NoptLocalSearch(_statespace, _last_best_state, 1);
            if (n_opt_local_search2.Run())
                _last_best_state = n_opt_local_search2.SolutionState;
        }

        #region attribs
        protected StateSpace _statespace;
        protected State _last_best_state;
        //protected int _swap_difference;
        #endregion
    }
}
