using System.Collections.Generic;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP;

namespace Logicx.Optimization.GenericStateSpace.StateSpaceInfo
{
    public struct CostMatrixElement
    {
        public CostMatrixElement(int row, int col, float value)
        {
            Row = row;
            Col = col;
            Value = value;
        }
        public override string ToString()
        {
            return "Row: " + Row + " Col: " + Col + " Value: " + Value;
        }
        public int Row;
        public int Col;
        public float Value;
    }

    public abstract class CostMatrix
    {
        public CostMatrix()
        {
        }

        public CostMatrix(List<Request> requests)
        {
            _requests = requests;
            InitMatrix();
        }

        /// <summary>
        /// if true all matrix elements will be stored in local list
        /// which is ordered ASC
        /// </summary>
        public bool WithOrderedValueList
		{
			set {
				_with_ordered_value_list = value;
			}
			
			get {
				return _with_ordered_value_list;
			}
		}

        public List<CostMatrixElement> OrderedValueList
        {
            set
            {
                _orderd_value_list = value;
            }

            get
            {
                return _orderd_value_list;
            }
        }
		
        //public List<Request> Requests
        //{
        //    set {
        //        _requests = value;
        //    }
			
        //    get {
        //        return _requests;
        //    }
        //}

        public abstract float this[int index_row, int index_col]
        {
            get;
        }

		
		/// <summary>
		/// Create the sturcture you need to calculate
        /// Cost Values
		/// </summary>
        public abstract void InitMatrix();

        protected List<Request> _requests;
        protected float[,] _costmatrix;
        protected bool _with_ordered_value_list = false;
        protected List<CostMatrixElement> _orderd_value_list;
    }
}
