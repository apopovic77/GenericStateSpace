using System;
using System.Collections.Generic;
using System.Text;
using System.Collections;
using Logicx.Gis.Routeplanning.RouteplannerLogic;
using Logicx.Optimization.Tourplanning.StateSpaceLogic.VRP;
using MathLib;

namespace Logicx.Optimization.GenericStateSpace.StateSpaceInfo
{
    public class TimeCongruencyMatrix : CostMatrix
    {
        public TimeCongruencyMatrix()
        {
        }

        public TimeCongruencyMatrix(List<Request> requests)
            : base(requests)
        {
        }

        public bool WithMedians
        {
            set
            {
                _with_medians = value;
            }

            get
            {
                return _with_medians;
            }
        }

        public bool WithAvgValues
        {
            set
            {
                _with_avgvalues = value;
            }

            get
            {
                return _with_avgvalues;
            }
        }

        public List<CostMatrixElement> Medians
        {
            set
            {
                _medians = value;
            }

            get
            {
                return _medians;
            }
        }

        public List<float> AvgValues
        {
            set
            {
                _avg_value = value;
            }

            get
            {
                return _avg_value;
            }
        }

        public float AvgValueOverall
        {
            set
            {
                _avg_value_overall = value;
            }

            get
            {
                return _avg_value_overall;
            }
        }

        /// <summary>
        /// get the time congruency value for row and col
        /// </summary>
        /// <param name="index_row"></param>
        /// <param name="index_col"></param>
        /// <returns></returns>
        public override float this[int index_row, int index_col]
        {
            get
            {
                return _costmatrix[index_row, index_col];
            }
        }


        /// <summary>
        /// Create the sturcture you need to calculate
        /// Cost Values
        /// </summary>
        public override void InitMatrix()
        {
            //create the cost matrix
            _costmatrix = new float[_requests.Count, _requests.Count];
            //create a list that saves the quality of the request in form of the calculated time congruency value
            sum_congruency_per_request = new float[_requests.Count];

            //create a list if required
            if (WithOrderedValueList)
                _orderd_value_list = new List<CostMatrixElement>();

            Median median = null;
            if (WithMedians)
            {
                _medians = new List<CostMatrixElement>();
                median = new Median();
            }
            float sum_congruency_all = 0;
            if (WithAvgValues)
                _avg_value = new List<float>();

            for (int index_row = 0; index_row < _requests.Count; index_row++)
            {
                float sum_congruency = 0;

                if (WithMedians)
                    median.Values.Clear();

                for (int index_col = 0; index_col < _requests.Count; index_col++)
                {
                    if (index_col == index_row)
                        continue;

                    //calc the costmatrix value --> time congruency
                    float time_congruency = GetTimeCongruencyIndex(_requests[index_row], _requests[index_col]);
                    _costmatrix[index_row, index_col] = time_congruency;

                    //save the value in the ordered list if required
                    if (WithOrderedValueList)
                        _orderd_value_list.Add(new CostMatrixElement(index_row, index_col, time_congruency));

                    if (WithMedians)
                        median.Values.Add(time_congruency);

                    //update sum congruency 
                    sum_congruency += time_congruency;


                    //cache best congruency value
                    if (_max_congruency_value < time_congruency)
                    {
                        _max_congruency_value = time_congruency;
                        _index_request_from_max_congruency = index_row;
                        _index_request_to_max_congruency = index_col;
                    }
                }

                if (WithAvgValues)
                    _avg_value.Add(sum_congruency / (_requests.Count - 1));
                if (WithMedians)
                    _medians.Add(new CostMatrixElement(index_row, -1, median.GetMedian()));


                //cache sum congruency values
                sum_congruency_per_request[index_row] = sum_congruency;
                sum_congruency_all += sum_congruency;

                //cahce best sum congruency value 
                if (_maxsum_congruency_value < sum_congruency)
                {
                    _maxsum_congruency_value = sum_congruency;
                    _index_request_maxsum_congruency = index_row;
                }
            }

            //calc overall avg value
            if (_requests.Count == 1)
            {
                _avg_value_overall = sum_congruency_all;
            }
            else
            {
                _avg_value_overall = sum_congruency_all / (_requests.Count * _requests.Count - _requests.Count);
            }

            //save the value in the ordered list if required
            if (WithOrderedValueList)
                _orderd_value_list.Sort(delegate (CostMatrixElement e1, CostMatrixElement e2) { return e1.Value.CompareTo(e2.Value); });

            if (WithMedians)
                _medians.Sort(delegate (CostMatrixElement e1, CostMatrixElement e2) { return e1.Value.CompareTo(e2.Value); });
        }

        public static float GetTimeCongruencyIndex(Request r1, Request r2)
        {
            TimeSpan r1_drt = r1.DRT;
            TimeSpan r2_drt = r2.DRT;

            // eval all 3 possible connected transportation forms
            Routeplan rp1 = _planner.GetRoutePlan(r1.FromUtm, r2.FromUtm);
            Routeplan rp2 = _planner.GetRoutePlan(r1.ToUtm, r2.ToUtm);
            Routeplan rp3 = _planner.GetRoutePlan(r1.ToUtm, r2.FromUtm);

            double min_ts = double.MaxValue;

            //check if possible to combine like that, only check pickup, delivery is direct
            if (r2.LPT > r1.EPT + rp1.TravelTime)
            {
                TimeSpan ts_p_1 = rp1.TravelTime + r2_drt + rp2.TravelTime;
                TimeSpan r2_p_waittime = new TimeSpan();
                if (r1.EPT + rp1.TravelTime < r2.EPT)
                    r2_p_waittime = r2.EPT - (r1.EPT + rp1.TravelTime);
                min_ts = (ts_p_1 + r2_p_waittime).TotalSeconds;
            }
            //check if possible to combine like that, check pickup and delivery
            if (r2.LPT > r1.EPT + rp1.TravelTime && r2.LDT < r1.LDT + rp2.TravelTime)
            {
                TimeSpan ts_p_2 = rp1.TravelTime + rp3.TravelTime + rp2.TravelTime;
                TimeSpan r2_p_waittime = new TimeSpan();
                if (r1.EPT + rp1.TravelTime < r2.EPT)
                    r2_p_waittime = r2.EPT - (r1.EPT + rp1.TravelTime);
                min_ts = (ts_p_2 + r2_p_waittime).TotalSeconds;
            }
            //check if possible to combine like that
            if (r2.LPT > r1.EPT + r1_drt + rp3.TravelTime)
            {
                TimeSpan ts_p_3 = r1_drt + rp3.TravelTime + r2_drt;
                TimeSpan r2_p_waittime = new TimeSpan();
                if (r1.EPT + r1_drt + rp3.TravelTime < r2.EPT)
                    r2_p_waittime = r2.EPT - (r1.EPT + r1_drt + rp3.TravelTime);
                min_ts = (ts_p_3 + r2_p_waittime).TotalSeconds;
            }

            return (float)(r1_drt.TotalSeconds / min_ts);
        }

        #region Attributes
        protected float[] sum_congruency_per_request;

        protected int _index_request_maxsum_congruency = -1;
        protected float _maxsum_congruency_value = float.MinValue;


        protected float _max_congruency_value = float.MinValue;
        protected int _index_request_from_max_congruency = -1;
        protected float _index_request_to_max_congruency = -1;

        protected bool _with_medians = true;
        protected bool _with_avgvalues = true;

        protected List<CostMatrixElement> _medians = null;
        protected List<float> _avg_value;
        protected float _avg_value_overall;
        private static Routeplanner _planner = RouteplannerFactory.GetPlanner();
        

        #endregion
    }
}
