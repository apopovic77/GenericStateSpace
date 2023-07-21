using System;
using System.Collections.Generic;
using System.Text;
//using ViHast.Persistence.Object;
//using RouteplanningLib.Routeplanner.SimplePlanner;
//using RouteplanningLib.RouteplannerLogic;
using System.Collections;

namespace Logicx.Optimization.Tourplanning.StateSpaceInfo
{
	public class DecentralizationIndex
	{
        //private DecentralizationIndex()
        //{}

        //public static DecentralizationIndex instance()
        //{
        //    return _instance;
        //}

        ///// <summary>
        ///// This method sets the DecentralizationIndex of each request according
        ///// to formula Dk in paper 'A new regret insertion heuristic for solving
        ///// large-scale dial-a-ride problems with time windows' (p. 12)
        ///// </summary>
        //public void CalculateDecentralizationIndex(List<Request> requests)
        //{
        //    _sumOfDecentralizationNums = 0;
        //    _decentralizationNumbers.Clear();

        //    foreach(Request r in requests)
        //    {
        //        double decentralizationNumber = CalculateOutsides(requests, r);
        //        _decentralizationNumbers.Add(r.Id, decentralizationNumber);
        //        _sumOfDecentralizationNums += decentralizationNumber;
        //    }
        //}


        ///// <summary>
        ///// Returns the outsides which can be built using the given
        ///// request as a pair with all other requests.
        ///// </summary>
        ///// <param name="r1"></param>
        ///// <returns></returns>
        //public double CalculateOutsides(List<Request> requests, Request r1)
        //{
        //    double decentralizationNumber = 0;
        //    foreach(Request r in requests)
        //    {
        //        float a = _planner.GetRoutePlan(r1.FromUtm, r.FromUtm).TravelDistance;
        //        float b = _planner.GetRoutePlan(r1.FromUtm, r.ToUtm).TravelDistance;
        //        float c = _planner.GetRoutePlan(r1.ToUtm, r.FromUtm).TravelDistance;
        //        float d = _planner.GetRoutePlan(r1.ToUtm, r.ToUtm).TravelDistance;

        //        decentralizationNumber += a + b + c + d;
        //    }

        //    return decentralizationNumber;
        //}

        ///// <summary>
        ///// Returns the DecentralizationIndex for
        ///// the given index.
        ///// </summary>
        ///// <param name="r"></param>
        ///// <returns></returns>
        //public double this[Request r]
        //{
        //    get
        //    {
        //        if (_decentralizationNumbers.Contains(r.Id))
        //            return ((double) _decentralizationNumbers[r.Id]) / _sumOfDecentralizationNums;
        //        return 0;
        //    }
        //}

        //private static Routeplanner _planner              = RouteplannerFactory.GetPlanner();
        //private static Hashtable _decentralizationNumbers = new System.Collections.Hashtable();
        //private static double _sumOfDecentralizationNums  = 0;
        //private static DecentralizationIndex _instance    = new DecentralizationIndex();
	}
}
