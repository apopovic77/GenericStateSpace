using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Utilities;


namespace Logicx.Optimization.GenericStateSpace
{
    public abstract class StateSpace
    {
        public enum Cachingstrategy
        {
            Depth_based_Caching = 1,
            Iteration_based_Caching = 2,
            Full_Caching = 3,
            No_Caching = 4,
            Normal_Caching = 5
        }	


        /// <summary>
        /// diese methode soll das initial_state object erstellen. dieses stellt die ausgangsbasis 
        /// für alle möglichen nächsten schritte
        /// </summary>
        public abstract void InitializeFirstState();

        /// <summary>
        /// Hier muss einen Contraint logic implementiert werden.
        /// Diese gewährleistet das die transition von dem gegebenen State zum nächsten durch die hier angeführte 
        /// Action möglich ist.
        /// </summary>
        /// <param name="action"></param>
        /// <returns></returns>
        protected abstract bool AssertBackwardConstraints(State state,Action action);

        protected abstract bool AssertForwardConstraints(State state, Action action);

        /// <summary>
        /// if you know which state you want to have, you should call this function.
        /// just provide the action that leads to the queried state and this method will do 
        /// the job for you
        /// </summary>
        /// <param name="state">the state from which you are transiting</param>
        /// <param name="action">the transiting action - you want to perform</param>
        /// <returns>returns null if the action cannot be performed</returns>
        public State NextState(State state, Action action) 
        {
            if (state == null)
                throw new Exception("state cannot be null");

            //get the action idex
            int action_index = _all_possible_actions.IndexOf(action);

            //see if the same state was queried the last time
            //if so, return it directly from the local cache
            if (state.LastQueriedNextState != null && state.LastQueriedNextState.Action == action)
                return state.LastQueriedNextState;

            //get the index of the action in the list
            if (!state.PossibleActionsIndex.Contains(action_index))
                return null;

            //check if the action is on the taboo list
            bool state_has_taboo_actions = state.HasTabooActions;
            if (state_has_taboo_actions && state.IsTabooAction(action_index))
                return null;

            if (state.NextStates != null)
            {
                if (action_index > state.NextStates.Count - 1)
                    action_index = state.NextStates.Count - 1;
                for (int i = action_index; i >= 0; i--)
                    if (state.NextStates[i].Action == action)
                        return state.NextStates[i];
                throw new Exception("error in logic, this cant be");
            }
            else 
            {
                //check constraint we dont know if this check has already been done, quit if contraint fails
                if (!AssertBackwardConstraints(state, action))
                    return null;
                if (_with_forward_constraint_checking && !AssertForwardConstraints(state, action))
                    return null;

                //create a clone of the actionslist
                MemoryEfficientSequentialIntegerList possible_actions_index_this_node = state.PossibleActionsIndex.Clone();

                //remove the transfering action from the new action list
                //as we have executed this step a new execution of this action in the next step 
                //is not needed anymore
                possible_actions_index_this_node.Remove(action_index);

                //remove all dependent actions
                List<Action> backward_dependent_actions = action.GetBackwardDependentActions();
                if (backward_dependent_actions != null)
                {
                    for (int j = 0; j < backward_dependent_actions.Count; j++)
                    {
                        possible_actions_index_this_node.Remove(_all_possible_actions.IndexOf(backward_dependent_actions[j]));
                    }
                }

                //create the new next state object
                State queried_state = CreateNewState(state, action, possible_actions_index_this_node);
                state.LastQueriedNextState = queried_state;
                _states_explored++;
                _states_existent++;
                return queried_state;
            }
        }


        /// <summary>
        /// gibt die zum state gehörigen nächsten States zurück
        /// </summary>
        /// <param name="this_state"></param>
        /// <returns></returns>
        public List<State> NextStates(State state)
        {
            //take the inital state if null was given
            if (state == null) {
                if (_initial_state == null)
                    InitializeFirstState();
                state = _initial_state;
            }

            if (state.NextStates != null)
                return state.NextStates;

            //perfrom for all possible next actions a transition -> the thereby achieved new state. and return it in a list
            //save the possible actions list here, so no property call has to be performed each time
            //Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index = state.PossibleActionsIndex;
            //create a list of new next states
            List<State> next_states = new List<State>();
            //does this state have taboo actions
            bool state_has_taboo_actions = state.HasTabooActions;
            for (int i = 0; i < state.PossibleActionsIndex.Count; i++)
            {
                //enable fast iteration through list
                if (i % 32 == 0 && state.PossibleActionsIndex.List[(int)Math.Floor(i / 32f)] == 0)
                {
                    i += 31;
                    continue;
                }
                if (!state.PossibleActionsIndex.Contains(i))
                    continue;


                //if this action is on the taboo list, than continue immediatelly, 
                //as this action does not lead to a solution
                if (state_has_taboo_actions && state.IsTabooAction(i))
                    continue;

                //get the action that is responsible for state transition
                Action transition_action = _all_possible_actions[i];

                //backward contraint checking
                //if that assertion failes, than no transition from the current state to the next state is possible
                //it is assumed here that this state transition is also not possible from any other state 
                if (!AssertBackwardConstraints(state, transition_action))
                {
                    // set action as taboo
                    state.SetActionTaboo(i);
                    // int this node this transition is not possible
                    // there this action does not lead to a new state
                    // maybe this action can be performed later
                    continue;
                }

                //forward contraint checking
                //if that assertion failes, than no transition from the current state to the next state is possible
                if (_with_forward_constraint_checking && !AssertForwardConstraints(state, transition_action))
                {
                    // set action as taboo
                    state.SetActionTaboo(i);
                    continue;
                }

                //create a clone of the actionslist
                Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index_this_node = state.PossibleActionsIndex.Clone();

                //remove the transfering action from the new action list
                //as we have executed this step a new execution of this action in the next step 
                //is not needed anymore
                possible_actions_index_this_node.Remove(i);

                //remove all dependent actions
                List<Action> backward_dependent_actions = transition_action.GetBackwardDependentActions();
                if (backward_dependent_actions != null)
                {
                    for (int j = 0; j < backward_dependent_actions.Count; j++)
                    {
                        possible_actions_index_this_node.Remove( _all_possible_actions.IndexOf(backward_dependent_actions[j]));
                    }
                }

                //überprüfe ob durch diese transition andere actions nicht mehr durchgeführt werden können
                //es wird quasi durch contraint validation jetzt überprüft welche der noch verbleiben possible 
                //actions in die possible list des nächsten states kommt, denn alle possible actions des nächsten states werde
                //hiermit als gültig angegeben, was bedeutet das jede transition von new_next_state mit transition irgendeiner 
                //possible_actions_this_node erlaubt ist.

                //create the new next state object
                State new_next_state = CreateNewState(state, transition_action, possible_actions_index_this_node);
                next_states.Add(new_next_state);
                _states_explored++;
                _states_existent++;
            }

            //do caching for generated next_states
            CacheStates(state, next_states);

            //return next_states
            return next_states;
        }
		
		/// <summary>
		/// Method CacheStates
		/// </summary>
		/// <param name="next_states">A  List<State></param>
		private void CacheStates(State state, List<State> next_states)
		{
            //save in a local counter how often this method has been called for this statespace
            _count_interations_next_states++;

            if (!_force_caching)
            {
                if (!_caching_enabled)
                    return;
                else if (_states_explored > _max_states_cached)
                    return;
                else if (_cachingstrategy == Cachingstrategy.Depth_based_Caching && state.DepthState > _count_cached_max_state_depth)
                    return;
                else if (_cachingstrategy == Cachingstrategy.Iteration_based_Caching && _count_interations_next_states > _count_cached_first_iterations)
                    return;
            }

            //save the new evaluated states list to the given state
            state.NextStates = next_states;
		}

        public virtual void AddCustomAction(Action action)
        {
            _custom_actions_count++;
            _all_possible_actions.Add(action);
            _initial_state.AddPossibleNextAction(action);
        }
		
		/// <summary>
		/// Method CreateNewState
		/// </summary>
		/// <param name="state">A  State</param>
		/// <param name="actions">A  List<int></param>
		/// <param name="possible_actions_this_node">A  List<int></param>
		/// <returns>A  State</retutns>
		protected abstract State CreateNewState(State prev_state, Action action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_this_node);

        /// <summary>
        /// durch eine aktion kann es sein das es neue actions gibt die zu erfüllen sind.
        /// diese methode sollte gewährleisten das actions für bestimmte situationen eingefügt werden wenn sie möglich werden.
        /// </summary>
        /// <param name="actions">An Action</param>
        /// <param name="possible_actions_this_node">A  List<int></param>
        protected abstract void UpdateNewPossibleActions(Action action, Logicx.Utilities.MemoryEfficientSequentialIntegerList possible_actions_index_this_node);

        #region Properties
        public bool WithForwardConstraintChecking
        {
            get { return _with_forward_constraint_checking; }
            set { _with_forward_constraint_checking = value; }

        }
        public int CustomActionsCount
        {
            set
            {
                _custom_actions_count = value;
            }

            get
            {
                return _custom_actions_count;
            }
        }
        public State InitialState
        {
            set
            {
                _initial_state = value;
            }

            get
            {
                if (_initial_state == null)
                    InitializeFirstState();
                return _initial_state;
            }
        }

        /// <summary>
        /// gibt die anzahl der Actions zurück die durchgeführt werden müssen um einen Lösungsstate zu erreichen.
        /// handelt es sich um eine problem ohne info soll hier bzw. wird -1 zurückgegeben werden
        /// </summary>
        public virtual int CountActions {
            get {
                return -1;
            }
        }


        /// <summary>
        /// gives info about how many states are alive, normaly this value is the same as States Explored,
        /// except backtracking has been done
        /// </summary>
        public ulong StatesExistent
        {
            set
            {
                _states_existent = value;
            }

            get
            {
                return _states_existent;
            }
        }

        /// <summary>
        /// Gives info about that count of states that have been created in this statespace
        /// </summary>
        public ulong StatesExplored
        {
            set
            {
                _states_explored = value;
            }

            get
            {
                return _states_explored;
            }
        }

        /// <summary>
        /// a list of all possible actions in the statespace
        /// </summary>
        public List<Action> AllPossibleActions
        {
            set
            {
                _all_possible_actions = value;
            }

            get
            {
                return _all_possible_actions;
            }
        }

        /// <summary>
        /// set this variable to true, if you want force the storage of next_states
        /// this overrules also the _max_States_cached
        /// </summary>
        public bool ForceCaching
        {
            set
            {
                _force_caching = value;
            }

            get
            {
                return _force_caching;
            }
        }
        /// <summary>
        /// sets the maximum states count to be cached, no matter which strategy is used
        /// if there are states to be cached which exceed this value, will not be cached
        /// </summary>
        public ulong CountMaxStatesCached
        {
            set
            {
                _max_states_cached = value;
            }

            get
            {
                return _max_states_cached;
            }
        }

        /// <summary>
        /// this local flag is used to indicate if caching for next states 
        /// should be done. this flag enables the cachingstrategy property
        /// </summary>
        public bool CachingEnabled
        {
            set
            {
                _caching_enabled = value;
            }

            get
            {
                return _caching_enabled;
            }
        }

        public ulong CountCachedIterations
        {
            set
            {
                _count_cached_first_iterations = value;
            }

            get
            {
                return _count_cached_first_iterations;
            }
        }

        public int CountCachedMaxStateDepth
        {
            set
            {
                _count_cached_max_state_depth = value;
            }

            get
            {
                return _count_cached_max_state_depth;
            }
        }
        /// <summary>
        /// Which caching strategy to be used
        /// </summary>
        public StateSpace.Cachingstrategy CachingStrategy
        {
            set
            {
                _cachingstrategy = value;
                switch (_cachingstrategy)
                {
                    case Cachingstrategy.Normal_Caching:
                    case Cachingstrategy.Depth_based_Caching:
                    case Cachingstrategy.Iteration_based_Caching:
                        _caching_enabled = true;
                        break;
                    case Cachingstrategy.Full_Caching:
                        _caching_enabled = true;
                        _force_caching = true;
                        break;
                    case Cachingstrategy.No_Caching:
                        _caching_enabled = false;
                        break;
                }
            }

            get
            {
                return _cachingstrategy;
            }
        }
        #endregion

        #region Attributes
        protected State _initial_state;
        protected ulong _states_explored = 0;
        protected ulong _states_existent = 0;
        /// <summary>
        /// this local flag is used to indicate if caching for next states 
        /// should be done. this flag overrules the cachingstrategy property
        /// </summary>
        protected bool _caching_enabled = true;

        /// <summary>
        /// sets the maximum states count to be cached, no matter which strategy is used
        /// if there are states to be cached which exceed this value, will not be cached
        /// </summary>
        protected ulong _max_states_cached = ulong.MaxValue;

        /// <summary>
        /// set this variable to true, if you want force the storage of next_states
        /// this overrules also the _max_States_cached
        /// </summary>
        protected bool _force_caching = false;

        /// <summary>
        /// the value of this var indicates how long caching should be done. 5 means after the first 5 iterations no more caching 
        /// will be performed
        /// </summary>
        protected ulong _count_cached_first_iterations = ulong.MaxValue;
        /// <summary>
        /// the value of this var indicates how long caching should be done. 5 means if the state depth level is smaller than 5 the states will be saved
        /// </summary>
        protected int _count_cached_max_state_depth = int.MaxValue;
        /// <summary>
        /// how often has the method next_states been called
        /// </summary>
        protected ulong _count_interations_next_states=0;
        /// <summary>
        /// Which caching strategy to be used for the statespace caching
        /// </summary>
        protected Cachingstrategy _cachingstrategy = Cachingstrategy.Normal_Caching;
        /// <summary>
        /// a list of all possible actions in the statespace
        /// </summary>
        protected List<Optimization.GenericStateSpace.Action> _all_possible_actions;
        /// <summary>
        /// if set to true alle actions must pass the forward constraint check
        /// </summary>
        protected bool _with_forward_constraint_checking = true;
        /// <summary>
        /// if the statespace is aware of custom actions
        /// than use this var and update the count for it
        /// </summary>
        protected int _custom_actions_count = 0;
        #endregion
    }
}
