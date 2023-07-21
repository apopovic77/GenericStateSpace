using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Logicx.Geo.Geometries;
using MathLib;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP
{

    public struct Vehicle
    {
        public enum ServicedAreaTypes
        {
            Serviced,
            NotServiced,
            PickupServiced,
            DeliveryServiced
        }

        public int capacity;
        public Vector2f CurrentPositionUtm { get; set; }
        public Vector2f DepotPositionUtm { get; set; }
        public SimplePolygon[] ServicedArea { get; set; }
        public ServicedAreaTypes[] ServicedAreaType { get; set; }
    }
}
