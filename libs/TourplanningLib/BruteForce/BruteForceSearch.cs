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
    public class BruteForceSearch
    {
        public BruteForceSearch(StateSpace statespace)
        {
            _statespace = statespace;
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

            //the last state known
            State curr_state = null;
            float min_cost = float.MaxValue;
            _solution_state = null;
            do
            {
                //fetch next states
                List<State> next_states = _statespace.NextStates(curr_state);
                if (next_states == null || next_states.Count == 0)
                {
                    if (curr_state.DepthState == 1)
                        break;
                    Backtrack(curr_state);
                    curr_state = curr_state.PreviousState;
                    continue;
                }

                curr_state = next_states[0];

                if (curr_state.DepthState >= _statespace.CountActions)
                {
                    if (curr_state.CurrentTargetValue < min_cost)
                    {
                        min_cost = curr_state.CurrentTargetValue;
                        _solution_state = curr_state;
                        if (NewBestSolutionState != null)
                            NewBestSolutionState(this, _solution_state);
                    }

                    Backtrack(curr_state);
                    curr_state = curr_state.PreviousState;
                }
            }
            while (true);

        }

        protected void Backtrack(State curr_state)
        {
            State prev_state = curr_state.PreviousState;
            prev_state.StateSpace = _statespace;
            prev_state.Remove(curr_state);
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
        protected State _solution_state;
        protected IDebugWriter _debugwriter;
        protected bool _with_second_chance = false;
        protected int _backtracking_base_count = 1000;
        protected bool _with_insertion_of_discarded_requests = false;
        public event EventHandler<State> NewBestSolutionState;
        #endregion
    }
}

