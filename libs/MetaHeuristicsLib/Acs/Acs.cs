using System;
using System.Collections.Generic;
using Logicx.Optimization.GenericStateSpace;
using System.Collections;
using Logicx.Optimization.GenericStateSpace.LocalSearch;
using Logicx.Optimization.MetaHeuristics.NearestNeighbour;
using Logicx.Utilities;

namespace Logicx.Optimization.MetaHeuristics.Acs
{
    public abstract class Acs
    {
        public Acs(StateSpace statespace)
        {
            //save statespace
            _statespace = statespace;

            //init matrices
            InitializeAcs();
        }

        public Acs(StateSpace statespace,State initalsolution)
        {
            //save statespace
            _statespace = statespace;

            //save the inital solution as the till now known best solution
            _best_solution_state = initalsolution;

            //init matrices
            InitializeAcs();
        }

        #region Properties
        public float Q
        {
            set
            {
                _q_value = value;
            }

            get
            {
                return _q_value;
            }
        }

        public ulong CountCachedIterations
        {
            set
            {
                _statespace.CountCachedIterations = value;
            }

            get
            {
                return _statespace.CountCachedIterations;
            }
        }

        public int CountCachedMaxStateDepth
        {
            set
            {
                _statespace.CountCachedMaxStateDepth = value;
            }

            get
            {
                return _statespace.CountCachedMaxStateDepth;
            }
        }

        /// <summary>
        /// Which caching strategy to be used
        /// </summary>
        public StateSpace.Cachingstrategy CachingStrategy
        {
            set
            {
                _statespace.CachingStrategy = value;
            }

            get
            {
                return _statespace.CachingStrategy;
            }
        }
        public ulong CountMaxStatesCached
        {
            set
            {
                _statespace.CountMaxStatesCached = value;
            }

            get
            {
                return _statespace.CountMaxStatesCached;
            }
        }
        public State SolutionState
        {
            set
            {
                _best_solution_state = value;
            }

            get
            {
                return _best_solution_state;
            }
        }
        #endregion

        #region Abstract Members
        protected abstract int NodeCount
        {
            get;
        }
        protected virtual int TrailMatrixDimensionRows
        {
            get
            {
                return NodeCount;
            }
        }
        protected virtual int TrailMatrixDimensionCols
        {
            get
            {
                return NodeCount;
            }
        }
		
		
		public int TotalIterations
		{
			set {
				_total_iterations = value;
			}
			
			get {
				return _total_iterations;
			}
		}
		
		public int AntPopulation
		{
			set {
				_ant_population = value;
			}
			
			get {
				return _ant_population;
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
		
		public bool WithBacktracking
		{
			set {
				_with_backtracking = value;
			}
			
			get {
				return _with_backtracking;
			}
		}
		
        /// <summary>
        /// If this is set to true, the ant will follow its path till it find a solution
        /// also when this would mean that we will not find a better solution
        /// </summary>
		public bool AllowExplorationBadPaths
		{
			set {
				_allow_exploration_bad_paths = value;
			}
			
			get {
				return _allow_exploration_bad_paths;
			}
		}
		
		public bool WithFastLocalsearchImprovment
		{
			set {
				_with_fast_localsearch_improvment = value;
			}
			
			get {
				return _with_fast_localsearch_improvment;
			}
		}
		
        protected abstract void InitMatrices();
        protected abstract void GetMatrixIndices(State state, out int i, out int j);
        #endregion

        #region ACS Initialization
        public void InitializeAcs()
        {
            if (_isInitialized) return;

            //set default values for control parameters
            _ant_population = 100;
            _total_iterations = 50;
            _evaporation_coefficient = 0.9f;
            _pheromone_influence = 0.9f;
            _attraction_influence = 1f;
            _q_value = 0.9f;
            _with_backtracking = false;
            _allow_exploration_bad_paths = true;
            _with_fast_localsearch_improvment = false;
            
            //set caching options
            _statespace.CachingStrategy = StateSpace.Cachingstrategy.No_Caching;
            //_statespace.CountCachedIterations = (ulong)_ant_population * 5;
            //_statespace.CountCachedMaxStateDepth = 5;
            _statespace.CountMaxStatesCached = 2500000;
            
            //run the nns and set the initial pheromon value
            InitDefaultPheromoneValue();

            //init the trail and attraction matrix
            CreateMatrices();

            //do a global update and update pheromone values on the path found through nns
            DoGlobalUpdate(_best_solution_state);

            _isInitialized = true;
        }

        /// <summary>
        /// Method InitMatrices
        /// </summary>
        private void CreateMatrices()
        {
            int matrix_dim_rows = TrailMatrixDimensionRows;
            int matrix_dim_cols = TrailMatrixDimensionCols;

            _attraction_matrix = new float[matrix_dim_rows, matrix_dim_cols];
            _trail_matrix = new float[matrix_dim_rows, matrix_dim_cols];

            InitMatrices();
        }

        /// <summary>
        /// Method InitInitialPheromone
        /// </summary>
        private void InitDefaultPheromoneValue()
        {
            //check if there is a solution given
            //if not than use the nearest neigbour search to solve the problem
            if (_best_solution_state == null) {
                //create an nns search, and find a solution
                _nns = new NearestNeighbourSearch(_statespace);
                _nns.Run();
                //save the nns solution as the till now known best solution
                _best_solution_state = _nns.SolutionState;
                if (_best_solution_state == null)
                    throw new Exception("Nearest Neighbour Search could not find a solution");
            }
                
            //set initial pheromon value
            _initial_pheromone_value = 1f / (NodeCount * _best_solution_state.CurrentTargetValue);
        }
        #endregion

        #region ACS ALGORITHM
        public void Run()
        {
            //State old_best=null;
            for (int iteration = 0; iteration < _total_iterations; iteration++)
            {
                                    int test = 0;

                //State final_best_ant_state = null;
                //do Ant WaySearch foreach ant in the population
                for (int ant = 0; ant < _ant_population; ant++)
                {
                    //init for the next ant waysearch the start_state --> which is null
                    State best_next_state = null;



                    //i th ant -> waysearch
                    do
                    {

                        test++;
                        //fetch next states
                        List<State> next_states = _statespace.NextStates(best_next_state);

                        //check if backtracking needed
                        //1. possible if no further tracking is possible but we still have not found a solution
                        //2. we have reached state where we know this state will never lead to better soluation
                        //   as the one we already have
                        if (next_states.Count == 0 || (!_allow_exploration_bad_paths && best_next_state != null && best_next_state.CurrentTargetValue > _best_solution_state.CurrentTargetValue))
                        {
                            if (best_next_state == null || best_next_state.PreviousState == null)
                            {
                                throw new Exception("No solution given, cannot evaluate optimal solution");
                                //return; //no solution given
                            }
                            //remove this state from the parent state
                            //we cannot find a solution following this path
                            best_next_state.PreviousState.Remove(best_next_state);
                            //indicate that a state has been deleted
                            _statespace.StatesExistent--;
                            //start again with the parent
                            //but this time without considering this state as a possible path
                            best_next_state = best_next_state.PreviousState;
                            if (_with_backtracking)
                                continue;
                            else
                                break;
                        }

                        //determine next node to be taken by ANT behaviour (=PseudoRandomSelection, RandomPropotionalSelection)
                        float rnd = GetRandomNumber();
                        if (rnd <= _q_value)
                            best_next_state = PseudoRandomSelection(next_states);
                        else
                            best_next_state = RandomProportionalSelection(next_states);

                        //do local update (also for edges that did not lead to a solution)
                        DoLocalUpdate(best_next_state);

                        //next_states = null;
                        //System.GC.Collect();


                    } while (best_next_state.DepthState < _statespace.CountActions);


                    if (best_next_state.DepthState == _statespace.CountActions)
                    {

                        if (_with_fast_localsearch_improvment)
                        {
                            LocalSearch ls = new LocalSearch(_statespace, best_next_state);
                            if (ls.Run())
                            {
                                WriteDebug("acs ls improvement: from " + best_next_state.CurrentTargetValue+ " to " + ls.SolutionState.CurrentTargetValue);
                                best_next_state = ls.SolutionState;
                            }
                        }



                        

                        //if (final_best_ant_state == null || final_best_ant_state.CurrTraveldistanceAll > best_next_state.CurrTraveldistanceAll)
                        //    final_best_ant_state = best_next_state;

                        if (best_next_state.CurrentTargetValue < _best_solution_state.CurrentTargetValue)
                        {
                            _best_solution_state = best_next_state;
                            WriteDebug("new final state " + _best_solution_state.CurrentTargetValue);
                        }
                        //else
                        //    WriteDebug("found solution " + best_next_state.CurrentTargetValue);
                    }
                }

                WriteDebug("finished iteration " + iteration);



                //if (final_best_ant_state != null)
                //    DoGlobalUpdate(final_best_ant_state);
                //else
                DoGlobalUpdate(_best_solution_state);



                //old_best = _best_solution_state;

            }
        }
        #endregion

        #region TrailMatrix Updateing Procedures

        /// <summary>
        /// The best ants path will update the trail matrix
        /// </summary>
        /// <param name="best_final_ant_state">A  State</param>
        public void DoGlobalUpdate(State state)
        {
            //update all edges with new pheromone values
            //that are part of the solution
            float pheromone = _evaporation_coefficient * (1f / state.CurrentTargetValue);//state.CurrTraveldistanceAll
            do
            {
                int i, j;
                GetMatrixIndices(state, out i, out j);

                //calc evaporization
                float evaporation = (1 - _evaporation_coefficient) * _trail_matrix[i, j];
                _trail_matrix[i, j] = evaporation + pheromone;
                if (j < TrailMatrixDimensionRows &&
                    i < TrailMatrixDimensionCols)
                    _trail_matrix[j, i] = evaporation + pheromone;


                state = state.PreviousState;
            } while (state.Action != null);
        }

        /// <summary>
        /// This Method is used in the Ant WaySearch for local updating choosen paths
        /// </summary>
        private void DoLocalUpdate(State state)
        {
            //update all edges with new pheromone values
            //that are part of the solution

            int i, j;
            GetMatrixIndices(state, out i, out j);
            float pheromone = _evaporation_coefficient * _initial_pheromone_value;
            float evaporation = (1 - _evaporation_coefficient) * _trail_matrix[i, j];
            _trail_matrix[i, j] = evaporation + pheromone;
            if (j < TrailMatrixDimensionRows &&
                i < TrailMatrixDimensionCols)
                _trail_matrix[j, i] = evaporation + pheromone;
        }
        #endregion

        #region Next Edge Determination
        private class ProductValueSorter : IComparer
        {
            int IComparer.Compare(Object x1, Object x2)
            {
                float productvalue1 = (float)((object[])x1)[0];
                float productvalue2 = (float)((object[])x2)[0];

                if (productvalue1 < productvalue2)
                    return -1;
                else if (productvalue1 == productvalue2)
                    return 0;
                else
                    return 1;
            }
        }
        private float GetRandomNumber()
        {
            float rndnumber = (float)_rnd.NextDouble();
            return rndnumber;
        }


        private State RandomProportionalSelection(List<State> next_states)
        {
            float product_value_sum = 0;
            float random_number = GetRandomNumber();

            ArrayList p_val_numerator = new ArrayList();
            ArrayList boundary = new ArrayList();
            ArrayList node_order = new ArrayList();

            int index_prev, index_curr;

            for (int i = 0; i < next_states.Count; i++)
            {
                GetMatrixIndices(next_states[i], out index_prev, out index_curr);

                object[] si = new object[2];
                si[0] = (float)(Math.Pow(_attraction_matrix[index_prev, index_curr], _attraction_influence) * Math.Pow(_trail_matrix[index_prev, index_curr], _pheromone_influence));
                si[1] = next_states[i];

                product_value_sum += (float)si[0];

                boundary.Add(si);
            }

            for (int i = 0; i < boundary.Count; i++)
                ((object[])boundary[i])[0] = (float)((object[])boundary[i])[0] / product_value_sum;

            boundary.Sort(new ProductValueSorter());

            float old_sum = 0;
            for (int i = 0; i < boundary.Count; i++)
            {
                old_sum += (float)((object[])boundary[i])[0];

                if (random_number <= old_sum)
                {
                    return (State)((object[])boundary[i])[1];
                }
            }
            return (State)((object[])boundary[boundary.Count - 1])[1];
        }

        private State PseudoRandomSelection(List<State> next_states)
        {
            float best_product = 0;
            State best_next_state = null;
            for (int i = 0; i < next_states.Count; i++)
            {
                int index_prev, index_curr;
                GetMatrixIndices(next_states[i], out index_prev, out index_curr);

                float product = (float)(Math.Pow(_attraction_matrix[index_prev, index_curr], _attraction_influence) * Math.Pow(_trail_matrix[index_prev, index_curr], _pheromone_influence));

                if (best_product < product)
                {
                    best_product = product;
                    best_next_state = next_states[i];
                }
            }
            return best_next_state;
        }
        #endregion

        #region Debug stuff
        public void WriteDebug(string msg) {
            if (_debugwriter != null)
                _debugwriter.WriteInfo(msg);
        }
        #endregion

        #region Attributes
        protected Random _rnd = new Random();
        protected NearestNeighbourSearch _nns;
        protected StateSpace _statespace;
        protected State _best_solution_state;
        protected float[,] _attraction_matrix;
        protected float[,] _trail_matrix;
        protected float _initial_pheromone_value;
        protected int _ant_population = 50, _total_iterations = 40;
        protected float _evaporation_coefficient;
        protected float _pheromone_influence;
        protected float _attraction_influence;
        protected float _q_value = 0.9f;
        protected bool _isInitialized=false;
        protected IDebugWriter _debugwriter;
        protected bool _with_backtracking;
        protected bool _allow_exploration_bad_paths;
        protected bool _with_fast_localsearch_improvment;
        #endregion
    }
}
