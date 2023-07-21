using System;
using System.Collections.Generic;
using System.Text;

namespace Logicx.Optimization.GenericStateSpace
{
    public abstract class Action
    {
		
		/// <summary>
		/// Method GetDependentActions
		/// </summary>
		/// <returns>A  List<Action></retutns>
		public List<Action> GetBackwardDependentActions()
		{
            return _backward_dependent_actions;
		}
        public List<Action> GetForwardDependentActions()
        {
            return _forward_dependent_actions;
        }
		/// <summary>
		/// Method SetDependency
		/// </summary>
        /// <param name="action_dependent">An Action from where we are dependent</param>
		public void SetBackwardDependency(Action action_dependent)
		{
            if (_backward_dependent_actions == null)
                _backward_dependent_actions = new List<Action>();

            if (!_backward_dependent_actions.Contains(action_dependent))
                _backward_dependent_actions.Add(action_dependent);
		}

        public void SetForwardDependency(Action action_dependent)
        {
            if (_forward_dependent_actions == null)
                _forward_dependent_actions = new List<Action>();

            if (!_forward_dependent_actions.Contains(action_dependent))
                _forward_dependent_actions.Add(action_dependent);
        }

        protected List<Action> _backward_dependent_actions;
        protected List<Action> _forward_dependent_actions;


        public override string ToString()
        {
            return base.ToString();
        }


    }
}
