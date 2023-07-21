using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Cache;
using System.Text;
using System.Threading.Tasks;
using Logicx.Gis.Routeplanning.RouteplannerLogic;
using MathLib;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{
    public class Request
    {
        public int passengers;
        public DateTime EPT;
        public DateTime LPT;
        public TimeSpan DRT;
        public TimeSpan LDT;
        public Vector2f FromUtm { get; set; }
        public Vector2f ToUtm { get; set; }
        public TimeSpan ServiceTime { get; set; }
    }

    public class DeliveryRequest : Request
    {
        public DateTime LDT;
        public DateTime LPT;
        public Vector2f ToUtm { get; set; }
        public Vector2f FromUtm { get; set; }

        public TimeSpan GetMRT(Routeplanner planner)
        {
            throw new NotImplementedException();
        }
    }


    public class PickupRequest : Request
    {
        public DateTime LDT;
        public Vector2f ToUtm { get; set; }

        public TimeSpan GetMRT(Routeplanner planner)
        {
            throw new NotImplementedException();
        }
    }
}
