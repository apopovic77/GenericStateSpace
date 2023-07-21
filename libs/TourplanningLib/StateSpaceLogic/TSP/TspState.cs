using System;
using System.Collections.Generic;
using System.Text;
using Logicx.Optimization.GenericStateSpace;
using Action = Logicx.Optimization.GenericStateSpace.Action;

namespace Logicx.Optimization.Tourplanning.StateSpaceLogic.TSP
{
    public class TspState : State
    {
		
		/// <summary>Creates a new object that is a copy of the current instance.</summary>
		/// <returns>A new object that is a copy of this instance.</returns>
		/// <filterpriority>2</filterpriority>
		public override State CloneShallow()
		{
			// TODO: Implement this method
			return null;
		}



        /// <summary>Creates a new object that is a copy of the current instance.</summary>
		/// <returns>A new object that is a copy of this instance.</returns>
		/// <filterpriority>2</filterpriority>
		public override Object Clone()
		{
			// TODO: Implement this method
			return null;
		}
		
		
		/// <summary>
		/// This Method evaluates if the action2 is dependent from action1
		/// </summary>
		/// <param name="_possible_actions">An Action</param>
		/// <param name="p1">An Action</param>
		/// <returns>A  bool</retutns>
		public override float CurrentTargetValue
		{
			get {
                return _curr_target_value;
			}
		}

        public float DistanceToNode {
            set {
                _curr_target_value = value;
            }
        }
		
		/// <summary>
		/// This Method evaluates if the action2 is dependent from action1
		/// </summary>
		/// <param name="_possible_actions">An Action</param>
		/// <param name="p1">An Action</param>
		/// <returns>A  bool</retutns>
		protected override bool IsActionBackwardDependent(Action action1, Action action2)
		{
			return false;
		}

        public override string ToString()
        {
            return "Depth "+_depth_state+" Action "+((TspAction)_action).IndexCity;
        }

        protected float _curr_target_value = 0;
    }
}
