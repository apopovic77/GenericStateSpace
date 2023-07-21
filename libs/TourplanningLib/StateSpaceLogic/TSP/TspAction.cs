using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using Action = Logicx.Optimization.GenericStateSpace.Action;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.TSP
{
    public class TspAction : Action
    {
        public TspAction(int index_city) {
            _index_city = index_city;
        }

        protected int _index_city=-1;
		
		public int IndexCity
		{
			set {
				_index_city = value;
			}
			
			get {
				return _index_city;
			}
		}

        public override bool Equals(Object obj)
        {
            //Check for null and compare run-time types.
            if ((obj == null) || !this.GetType().Equals(obj.GetType()))
            {
                return false;
            }
            else
            {
                TspAction other_action = (TspAction)obj;
                return this._index_city == other_action._index_city;
            }
        }

        public static bool operator ==(TspAction obj1, TspAction obj2)
        {
            if (ReferenceEquals(obj1, obj2))
            {
                return true;
            }

            if (ReferenceEquals(obj1, null))
            {
                return false;
            }

            if (ReferenceEquals(obj2, null))
            {
                return false;
            }

            return obj1._index_city == obj2._index_city;

        }

        public static bool operator !=(TspAction action1, TspAction action2)
        {
            return !(action1 == action2);
        }

        public override string ToString()
        {
            return "Stadt Index: "+_index_city;
        }
    }
}
