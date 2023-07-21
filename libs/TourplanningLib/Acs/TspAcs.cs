using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.TSP;
using Logicx.Optimization.GenericStateSpace;
using MathLib;
using Logicx.Utilities;

namespace Logicx.Optimization.Tourplanning.ACS
{
    public class TspAcs : Acs
    {
        public TspAcs()
        {
        }
        public TspAcs(TspStateSpace statespace)
            : base(statespace)
        {
        }
		     
		protected override int NodeCount
		{
			get {
                return ((TspStateSpace)_statespace).Cities.Length;
			}
		}
		
		protected override int TrailMatrixDimensionRows
		{
			get {
                return NodeCount;
			}
		}
		
		protected override int TrailMatrixDimensionCols
		{
			get {
                return NodeCount;
			}
		}
		
		protected override void InitMatrices()
		{
            Vector2f[] cities = ((TspStateSpace)_statespace).Cities;
            Vector2f v1, v2;
            for (int j = 0; j < cities.Length; j++)
            {
                for (int k = 0; k < cities.Length; k++)
                {
                    v1 = cities[j];
                    v2 = cities[k];
                    _attraction_matrix[j,k] = 1 / ((Vector2f)v1 - v2).GetLen();
                    _trail_matrix[j,k] = _initial_pheromone_value;
                }
            }
		}
		
		protected override bool GetMatrixIndices(State state, out int i, out int j)
		{
            
            TspState tspstate = (TspState)state;
            TspAction tspaction = (TspAction)tspstate.Action;
            j = tspaction.IndexCity;
            i = -1;
            if (tspstate.PreviousState == null)
                return false;
            TspAction tspaction_prev = (TspAction)tspstate.PreviousState.Action;
            i = (tspaction_prev != null) ? tspaction_prev.IndexCity : tspaction.IndexCity;
            return true;
        }


        
    }
}
