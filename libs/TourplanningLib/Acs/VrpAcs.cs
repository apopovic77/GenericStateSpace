using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Gis.Routeplanning.RouteplannerLogic;
using Logicx.Optimization.GenericStateSpace;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP;
using MathLib;

namespace Logicx.Optimization.Tourplanning.ACS
{
    public class VrpAcs : Acs
    {
		
		
        public VrpAcs(VrpStateSpace statespace) : base(statespace)
        {
        }
        public VrpAcs(VrpStateSpace statespace, VrpState initalsolution) : base(statespace,initalsolution)
        {
        }       
		     
		protected override int NodeCount
		{
			get {
                return ((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).Vehicles.Count + ((VrpStateSpace)_statespace).CustomActionsCount;
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
                return ((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).CustomActionsCount;
			}
		}
		
		protected override void InitMatrices()
		{
            Routeplanner planner = ((VrpStateSpace)_statespace).Planner;
            Vector2f v1, v2;

            for (int k = 0; k < ((VrpStateSpace)_statespace).CustomActionsCount; k++)
            {
                for (int x = 0; x < ((VrpStateSpace)_statespace).CustomActionsCount; x++)
                {
                    v1 = ((VrpStopAction)_statespace.AllPossibleActions[((VrpStateSpace)_statespace).Requests.Count * 2 * ((VrpStateSpace)_statespace).Vehicles.Count + k]).StopPosition;
                    v2 = ((VrpStopAction)_statespace.AllPossibleActions[((VrpStateSpace)_statespace).Requests.Count * 2 * ((VrpStateSpace)_statespace).Vehicles.Count + x]).StopPosition;

                    _attraction_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + k, ((VrpStateSpace)_statespace).Requests.Count * 2 + x] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;
                    _trail_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + k, ((VrpStateSpace)_statespace).Requests.Count * 2 + x] = _initial_pheromone_value;
                }
            }


            for (int j = 0; j < ((VrpStateSpace)_statespace).Requests.Count; j++)
            {

                for (int k = 0; k < ((VrpStateSpace)_statespace).CustomActionsCount; k++)
                {
                    v1 = ((VrpStopAction)_statespace.AllPossibleActions[((VrpStateSpace)_statespace).Requests.Count * 2 * ((VrpStateSpace)_statespace).Vehicles.Count + k]).StopPosition;
                    v2 = ((VrpStateSpace)_statespace).Requests[j].FromUtm;
                    _attraction_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + k, j * 2] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;
                    _attraction_matrix[j * 2, ((VrpStateSpace)_statespace).Requests.Count * 2 + k] = 1 / (float)planner.GetRoutePlan(v2, v1).TravelTime.TotalSeconds;


                    v2 = ((VrpStateSpace)_statespace).Requests[j].ToUtm;
                    _attraction_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + k, j * 2 + 1] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;
                    _attraction_matrix[j * 2 + 1, ((VrpStateSpace)_statespace).Requests.Count * 2 + k] = 1 / (float)planner.GetRoutePlan(v2, v1).TravelTime.TotalSeconds;

                    _trail_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + k, j * 2] = _initial_pheromone_value;
                    _trail_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + k, j * 2 + 1 ] = _initial_pheromone_value;
                    _trail_matrix[j * 2,((VrpStateSpace)_statespace).Requests.Count * 2 + k] = _initial_pheromone_value;
                    _trail_matrix[j * 2 + 1,((VrpStateSpace)_statespace).Requests.Count * 2 + k] = _initial_pheromone_value;


                }


                for (int k = 0; k < ((VrpStateSpace)_statespace).Vehicles.Count; k++)
                {
                    v1 = ((VrpStateSpace)_statespace).Vehicles[k].DepotPositionUtm;
                    v2 = ((VrpStateSpace)_statespace).Requests[j].FromUtm;
                    _attraction_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).CustomActionsCount + k, j * 2] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;
                    _trail_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).CustomActionsCount + k, j * 2] = _initial_pheromone_value;
                    
                    for (int x = 0; x < ((VrpStateSpace)_statespace).CustomActionsCount; x++)
                    {
                        v2 = ((VrpStopAction)_statespace.AllPossibleActions[((VrpStateSpace)_statespace).Requests.Count * 2 * ((VrpStateSpace)_statespace).Vehicles.Count + x]).StopPosition;

                        _attraction_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).CustomActionsCount + k, ((VrpStateSpace)_statespace).Requests.Count * 2 + x] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;
                        _trail_matrix[((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).CustomActionsCount + k, ((VrpStateSpace)_statespace).Requests.Count * 2 + x ] = _initial_pheromone_value;
                    }
                }

                for (int i = 0; i < ((VrpStateSpace)_statespace).Requests.Count; i++)
                {
                    if (i != j)
                    {
                        v1 = ((VrpStateSpace)_statespace).Requests[i].FromUtm;
                        v2 = ((VrpStateSpace)_statespace).Requests[j].FromUtm;
                        _attraction_matrix[i * 2, j * 2] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;

                        v1 = ((VrpStateSpace)_statespace).Requests[i].ToUtm;
                        v2 = ((VrpStateSpace)_statespace).Requests[j].ToUtm;
                        _attraction_matrix[i * 2 + 1, j * 2 + 1] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;
                    }

                    v1 = ((VrpStateSpace)_statespace).Requests[i].FromUtm;
                    v2 = ((VrpStateSpace)_statespace).Requests[j].ToUtm;
                    _attraction_matrix[i * 2, j * 2 + 1] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;

                    v1 = ((VrpStateSpace)_statespace).Requests[i].ToUtm;
                    v2 = ((VrpStateSpace)_statespace).Requests[j].FromUtm;
                    _attraction_matrix[i * 2 + 1, j * 2] = 1 / (float)planner.GetRoutePlan(v1, v2).TravelTime.TotalSeconds;


                    _trail_matrix[i * 2, j * 2] = _initial_pheromone_value;
                    _trail_matrix[i * 2 + 1, j * 2 + 1] = _initial_pheromone_value;
                    _trail_matrix[i * 2 + 1, j * 2] = _initial_pheromone_value;
                    _trail_matrix[i * 2, j * 2 + 1] = _initial_pheromone_value;

                }
            }


		}
		
		protected override bool GetMatrixIndices(State state, out int i, out int j)
		{
            i = -1;
            j = -1;

            j = GetJIndex(state);

            VrpState prev_state = (VrpState)state.PreviousState;
            if (prev_state.LastServedRequestIndex[((VrpAction)state.Action).VehicleIndex] == -1)
                i = ((VrpStateSpace)_statespace).Requests.Count * 2 + ((VrpStateSpace)_statespace).CustomActionsCount + ((VrpAction)state.Action).VehicleIndex;
            else
                i = GetIIndex(state);

            return true;
        }

        private int GetIIndex(State state)
        {
            VrpStateSpace vrp_statespace = (VrpStateSpace)_statespace;
            int index = -1;

            if (state.PreviousState.Action is VrpRequestAction)
            {
                index = ((VrpState)state.PreviousState).LastServedRequestIndex[((VrpAction)state.Action).VehicleIndex] * 2;
                if (!((VrpState)state.PreviousState).LastServedRequestIspickup[((VrpAction)state.Action).VehicleIndex])
                    index++;
            }
            else if (state.PreviousState.Action is VrpStopAction)
            {
                int first_custom_action = vrp_statespace.Requests.Count * 2 * vrp_statespace.Vehicles.Count;
                //XBUG: DIE FORSCHLEIFE ERSETZEN DURCH EINE INTELLIGENTERE LOGIK, PERFORMANCE ISSUE
                for (int k = 0; k < vrp_statespace.AllPossibleActions.Count; k++)
                {
                    if (vrp_statespace.AllPossibleActions[k] == state.PreviousState.Action)
                    {
                        index = vrp_statespace.Requests.Count * 2 + (k - first_custom_action);
                        break;
                    }
                }

                if (index == -1)
                    throw new Exception("somethings wrong here");
            }
            else
            {
                throw new Exception("unknown action type");
            }

            return index;
        }

        protected int GetJIndex(State state)
        {
            VrpStateSpace vrp_statespace = (VrpStateSpace)_statespace;
            int index = -1;

            if (state.Action is VrpRequestAction)
            {
                index = ((VrpRequestAction)state.Action).RequestIndex * 2;
                if (!((VrpRequestAction)state.Action).IsPickup)
                    index++;
            }
            else if (state.Action is VrpStopAction)
            {
                int first_custom_action = vrp_statespace.Requests.Count * 2 * vrp_statespace.Vehicles.Count;
                //XBUG: DIE FORSCHLEIFE ERSETZEN DURCH EINE INTELLIGENTERE LOGIK, PERFORMANCE ISSUE
                for (int k = 0; k < vrp_statespace.AllPossibleActions.Count; k++)
                {
                    if (vrp_statespace.AllPossibleActions[k] == state.Action)
                    {
                        index = vrp_statespace.Requests.Count * 2 + (k - first_custom_action);
                        break;
                    }
                }

                if (index == -1)
                    throw new Exception("somethings wrong here");
            }
            else
            {
                throw new Exception("unknown action type");
            }

            return index;
        }
    }
}
