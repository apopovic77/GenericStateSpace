using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Utilities;

namespace Logicx.Optimization.GenericStateSpace
{
    /// <summary>
    /// Ein State beschreibt einen Zustand im Zustandsraum. Der Zustandsraum legt fest welche Aktionen möglich sind.
    /// Nicht jede Aktion muss zu einer Lösung führen. Der Zustandsraum gibt ledigliche darüber aufschluss welche Aktionen durchgeführt werden können.
    /// Diese Art der Definition eines Zustandsraum soll die implizite Defintion alle Möglichen Aktionen für ein bestimmtes Problem realisieren.
    /// Diese Klasse dient dann als ausgangsbasis für verschiedene Algorithmen deren Ziel die Suche von Lösungen im Zustandsraum ist. 
    /// </summary>
    public abstract class State : ICloneable
    {
        #region construction
        public State() {
            _prev_state = null;
            //_possible_actions_index = new MemoryEfficientSequentialIntegerList();
            
        }
        public State(State prev_state, Action action, MemoryEfficientSequentialIntegerList possible_actions_index, StateSpace statespace) {
            _prev_state = prev_state;
            _possible_actions_index = possible_actions_index;
            _action = action;
            _statespace = statespace;
        }
        #endregion		

        #region Taboo Logic
        /// <summary>
        /// Method Remove
        /// </summary>
        /// <param name="best_next_state">A  State</param>
        public void Remove(State state)
        {
            int index_action = _statespace.AllPossibleActions.IndexOf(state.Action);
            SetActionTaboo(index_action);

            if (_next_states != null)
            {
                _next_states.Remove(state);
            }
        }

        /// <summary>
        /// call this method if you want to set an method as taboo.
        /// this means the exectuion of this action is not allowed in this state
        /// </summary>
        /// <param name="i">An int</param>
        public void SetActionTaboo(int index_action)
        {
            //update taboo list
            if (_taboolist == null)
            {
                int buffersize = (int)Math.Ceiling(PossibleActionsIndex.Count / 32f);
                _taboolist = new uint[buffersize];
            }

            int index_buffer = (int)Math.Floor(index_action / 32f);
            int index_bit = index_action % 32;

            _taboolist[index_buffer] += (uint)(1 << index_bit);
        }
        public bool IsTabooAction(int index)
        {
            int index_buffer = (int)Math.Floor(index / 32f);
            int index_bit = index % 32;
            uint check_val = _taboolist[index_buffer] & (uint)(1 << index_bit);

            if (check_val > 0)
                return true;
            else
                return false;
        }
        public bool HasTabooActions {
            get {
                if (_taboolist != null)
                    return true;
                else
                    return false;
            }
        }
        #endregion

        /// <summary>
        /// Mit dieser Methode kann möglicher neuer State eingefügt werden
        /// </summary>
        /// <param name="action">A  VrpAction</param>
        public void AddPossibleNextAction(Action action)
        {
            int index_of = _statespace.AllPossibleActions.IndexOf(action);
            _possible_actions_index.Add(index_of);
        }

        /// <summary>
        /// Diese Methode gewährleistet den verbund von Aktionen die im Zustandraum stattfinden können
        /// diese Abhängigkeit die hier direkt im action selbst gespeichert wird, dient der performance steigerung.
        /// Diese Methode sollte nur einmal beim initialisieren aufgerufen werden, nachdem alle actions die möglich sind
        /// erfasst wurden. 
        /// </summary>
        public virtual void SetActionDependencies()
        {
            for (int i = 0; i < _possible_actions_index.Count; i++)
                for (int j = 0; j < _possible_actions_index.Count; j++)
                    if (i != j)
                    {
                        if (IsActionBackwardDependent(_statespace.AllPossibleActions[ i ], _statespace.AllPossibleActions[ j ]))
                            _statespace.AllPossibleActions[ i ].SetBackwardDependency(_statespace.AllPossibleActions[ j ]);

                        if (IsActionBackwardDependent(_statespace.AllPossibleActions[ j ], _statespace.AllPossibleActions[ i ]))
                            _statespace.AllPossibleActions[ j ].SetBackwardDependency(_statespace.AllPossibleActions[  i ]);
                    }
        }

        /// <summary>
        /// This Method evaluates if the action2 is dependent from action1
        /// </summary>
        /// <param name="action1">An Action</param>
        /// <param name="action2">An Action</param>
        /// <returns>A  bool</retutns>
        protected abstract bool IsActionBackwardDependent(Action action1, Action action2);


        /// <summary>
        /// return a list of actions in correct order
        /// </summary>
        /// <returns></returns>
        public List<Action> ToActionList()
        {
            if (_action == null)
                return null;

            List<Action> action_list = new List<Action>(_depth_state);
            action_list.Add(_action);
            State current_state = this.PreviousState;

            do {
                action_list.Insert(0,current_state.Action);
                current_state = current_state.PreviousState;
            } while ( current_state != null);

            return action_list;
            

        }



        /// <summary>Creates a new object that is a copy of the current instance.</summary>
        /// <returns>A new object that is a copy of this instance.</returns>
        /// <filterpriority>2</filterpriority>
        /// 

        public abstract Object Clone();
        public abstract State CloneShallow();

        #region Properties
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
        public abstract float CurrentTargetValue
        {
            get;
        }

        public State LastQueriedNextState
        {
            set
            {
                _lastqueried_next_state = value;
            }

            get
            {
                return _lastqueried_next_state;
            }
        }
        /// <summary>
        /// gibt aufschluss über tiefe des state im stateraum, oder anders ausgedrückt
        /// durch wieviele aktionen wurde dieser state erreicht
        /// </summary>
        public int DepthState
        {
            set
            {
                _depth_state = value;
            }

            get
            {
                return _depth_state;
            }
        }

        public List<State> NextStates
        {
            set
            {
                _next_states = value;
            }

            get
            {
                return _next_states;
            }
        }

        public Action Action
        {
            set
            {
                _action = value;
            }

            get
            {
                return _action;
            }
        }
        /// <summary>
        /// sets or gets a possible actionlist. setting possible actionslists
        /// should only be done in inital phases.
        /// </summary>
        public MemoryEfficientSequentialIntegerList PossibleActionsIndex
        {
            set
            {
                _possible_actions_index = value;
            }

            get
            {
                return _possible_actions_index;
            }
        }

        public State PreviousState
        {
            set
            {
                _prev_state = value;
            }

            get
            {
                return _prev_state;
            }
        }

        #endregion

        #region Attributes
        /// <summary>
        /// eine menge aller möglichen aktionen die in diesem state ausgeführt werden können
        /// eine aktion dieser liste führt zum nächsten state
        /// </summary>
        //protected List<int> _possible_actions_index;
        protected MemoryEfficientSequentialIntegerList _possible_actions_index;

        /// <summary>
        /// diese aktion ist ausschlaggebend für dieses state, diese aktion ist somit für die transition verantwortlich
        /// in verbindung mit den previous states kann nachvollzogen werden welche liste an aktionen zu diesem state geführt hat.
        /// </summary>
        protected Action _action;

        /// <summary>
        /// previous State speichert den letzten State.
        /// hiermit wird das backtracking ermöglicht
        /// </summary>
        protected State _prev_state;

        /// <summary>
        /// this var stores all possible next states
        /// </summary>
        protected List<State> _next_states;

        /// <summary>
        /// gibt aufschluss über tiefe des state im stateraum, oder anders ausgedrückt
        /// durch wieviele aktionen wurde dieser state erreicht
        /// </summary>
        protected int _depth_state = 0;

        /// <summary>
        /// saves bitwise if a next_action is taboo (bzw. if a next action does not lead to a solution)
        /// </summary>
        protected uint[] _taboolist;
        /// <summary>
        /// if one next_state was called this one next_state will be stored in here
        /// </summary>
        protected State _lastqueried_next_state;

        /// <summary>
        /// a reference to the statespace that which we are belonging to
        /// </summary>
        protected StateSpace _statespace;
        #endregion 
    }
}
