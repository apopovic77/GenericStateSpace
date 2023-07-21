using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using MathLib;
using Logicx.Utilities;
using Action = Logicx.Optimization.GenericStateSpace.Action;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.TSP
{
    public class TspStateSpace : StateSpace
    {
        public TspStateSpace(Vector2f[] cities, int index_startnode)
        {

            _cities = cities;
            _index_startnode = index_startnode;
            _with_forward_constraint_checking = false;
            _crosslines_matrix = new bool[cities.Length, cities.Length, cities.Length, cities.Length];

            for (int i = 0; i < cities.Length; i++)
            {
                for (int j = 0; j < cities.Length; j++)
                {
                    for (int k = 0; k < cities.Length; k++)
                    {
                        for (int l = 0; l < cities.Length; l++)
                        {
                            Vector2f city_1 = Cities[i];
                            Vector2f city_2 = Cities[j];
                            Vector2f city_3 = Cities[k];
                            Vector2f city_4 = Cities[l];

                            Edge edge = new Edge((Vector2d)city_1, (Vector2d)city_2);
                            Edge edge2 = new Edge((Vector2d)city_3, (Vector2d)city_4);
                            Vector2d cutpoint;
                            _crosslines_matrix[i, j, k, l] = LinearAlgebra.GetCutPointInSegment(edge2, edge, out cutpoint);
                        }
                    }

                }
            }
        }

        public Vector2f[] Cities
        {
            set
            {
                _cities = value;
            }

            get
            {
                return _cities;
            }
        }


        /// <summary>
        /// diese methode soll das initial_state object erstellen. dieses stellt die ausgangsbasis
        /// für alle möglichen nächsten schritte
        /// </summary>
        public override void InitializeFirstState()
        {
            if (_initial_state != null)
                return;

            _initial_state = new TspState();
            _initial_state.PreviousState = null;
            _initial_state.Action = new TspAction(_index_startnode);
            _initial_state.StateSpace = this;
            _initial_state.DepthState = 1;
            _initial_state.PossibleActionsIndex = new MemoryEfficientSequentialIntegerList(_cities.Length);

            _all_possible_actions = new List<Action>();
            for (int i = 0; i < _cities.Length; i++)
            {
                _all_possible_actions.Add(new TspAction(i));
            }

            //_initial_state.AddPossibleNextAction(new TspAction(_index_startnode));
            for (int i = 0; i < _cities.Length; i++)
            {
                _initial_state.AddPossibleNextAction(new TspAction(i));
            }
        }

        protected override bool AssertBackwardConstraints(State state, Action action)
        {
            TspAction tspaction = (TspAction)action;

            if (state.DepthState == 0 && tspaction.IndexCity != _index_startnode)
                return false;
            else if (state.DepthState > 0 && state.DepthState < _cities.Length && tspaction.IndexCity == _index_startnode)
                return false;
            else if (state.DepthState == _cities.Length && tspaction.IndexCity != _index_startnode)
                return false;
            else
                return true;
        }

        protected override bool AssertForwardConstraints(State state, Action action)
        {
            int city_index_nextedge_1 = ((TspAction)state.Action).IndexCity;
            int city_index_nextedge_2 = ((TspAction)action).IndexCity;

            TspAction action_next = (TspAction)state.Action;
            TspState state_prev = (TspState)state.PreviousState;

            while (state_prev != null)
            {
                int city_index_prev_1 = ((TspAction)state_prev.Action).IndexCity;
                int city_index_prev_2 = action_next.IndexCity;

                if (_crosslines_matrix[city_index_nextedge_1, city_index_nextedge_2, city_index_prev_1, city_index_prev_2])
                    return false;

                action_next = (TspAction)state_prev.Action;
                state_prev = (TspState)state_prev.PreviousState;
            }

            return true;
        }

        protected override State CreateNewState(State prev_state, Action action, MemoryEfficientSequentialIntegerList possible_actions_this_node)
        {
           
            TspAction tsp_action = (TspAction)action;

            float len = 0;//(_cities[tsp_action.IndexCity] - _cities[((TspAction)prev_state.Action).IndexCity]).GetLen();
            if (prev_state.Action != null)
                len = (_cities[((TspAction)prev_state.Action).IndexCity] - _cities[((TspAction)action).IndexCity]).GetLen();
            TspState state = new TspState();
            state.Action = action;
            state.PossibleActionsIndex = possible_actions_this_node;
            state.DepthState = prev_state.DepthState + 1;
            state.DistanceToNode = prev_state.CurrentTargetValue + len;
            state.PreviousState = prev_state;
            return state;
        }

        protected override void UpdateNewPossibleActions(Action action, MemoryEfficientSequentialIntegerList possible_actions_index_this_node)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// how many actions to we have till we find a solution
        /// </summary>
        public override int CountActions
        {
            get
            {
                return _cities.Length + 1;
            }
        }


        #region Attributes
        protected Vector2f[] _cities;
        protected int _index_startnode;
        protected bool[,,,] _crosslines_matrix;
        #endregion
    }
}
