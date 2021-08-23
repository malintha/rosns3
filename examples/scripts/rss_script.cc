/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 Timo Bingmann
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Timo Bingmann <timo.bingmann@student.kit.edu>
 */

#include "ns3/propagation-loss-model.h"
#include "ns3/jakes-propagation-loss-model.h"
#include "ns3/constant-position-mobility-model.h"

#include "ns3/config.h"
#include "ns3/command-line.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include <map>

using namespace ns3;

/// Round a double number to the given precision. e.g. dround(0.234, 0.1) = 0.2
/// and dround(0.257, 0.1) = 0.3
static double dround (double number, double precision)
{
  number /= precision;
  if (number >= 0)
    {
      number = floor (number + 0.5);
    }
  else
    {
      number = ceil (number - 0.5);
    }
  number *= precision;
  return number;
}

static Gnuplot TestDeterministic (Ptr<PropagationLossModel> model)
{
  Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
  Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();

  Gnuplot plot;

  plot.AppendExtra ("set xlabel 'Distance'");
  plot.AppendExtra ("set ylabel 'rxPower (dBm)'");
  plot.AppendExtra ("set key top right");

  double txPowerDbm = +16.0206; // dBm

  Gnuplot2dDataset dataset;

  dataset.SetStyle (Gnuplot2dDataset::LINES);

  {
    a->SetPosition (Vector (0.0, 0.0, 0.0));

    for (double distance = 1.0; distance < 250.0; distance += 1.0)
      {
        b->SetPosition (Vector (distance, 0.0, 0.0));

        // CalcRxPower() returns dBm.
        double rxPowerDbm = model->CalcRxPower (txPowerDbm, a, b);

        dataset.Add (distance, rxPowerDbm);

        Simulator::Stop (Seconds (1.0));
        Simulator::Run ();
      }
  }

  std::ostringstream os;
  os << "txPower " << txPowerDbm << "dBm";
  dataset.SetTitle (os.str ());

  plot.AddDataset (dataset);

  // plot.AddDataset ( Gnuplot2dFunction ("-94 dBm CSThreshold", "-94.0") );

  return plot;
}

static Gnuplot TestProbabilistic (Ptr<PropagationLossModel> model, unsigned int samples = 100)
{
  Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
  Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();

  Gnuplot plot;

  plot.AppendExtra ("set xlabel 'Distance'");
  plot.AppendExtra ("set ylabel 'rxPower (dBm)'");

  double txPowerDbm = +16.0206; // dBm

  Gnuplot2dDataset dataset;
  typedef std::map<double, unsigned int> rxPowerMapType;

  // Take given number of samples from CalcRxPower() and show probability
  // density for discrete distances.
  {
    a->SetPosition (Vector (0.0, 0.0, 0.0));

    for (double distance = 1.0; distance < 200.0; distance += 1.0)
      {
        b->SetPosition (Vector (distance, 0.0, 0.0));

        for (unsigned int samp = 0; samp < samples; ++samp)
          {
            // CalcRxPower() returns dBm.
            double rxPowerDbm = model->CalcRxPower (txPowerDbm, a, b);
            // rxPowerDbm = dround (rxPowerDbm, 1.0);

            // rxPowerMap[rxPowerDbm]++;
            dataset.Add (distance, rxPowerDbm);
            
            Simulator::Stop (Seconds (0.01));
            Simulator::Run ();
          }

        dataset.AddEmptyLine ();
      }
  }

  std::ostringstream os;
  os << "txPower " << txPowerDbm << "dBm";
  dataset.SetTitle (os.str ());

  plot.AddDataset (dataset);

  return plot;
}

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  
  GnuplotCollection gnuplots ("main-propagation-loss.pdf");

  // {
  //   Ptr<PropagationLossModel> log = CreateObject<LogDistancePropagationLossModel> ();

  //   Gnuplot plot = TestDeterministic (log);
  //   plot.SetTitle ("ns3::LogDistancePropagationLossModel");
  //   gnuplots.AddPlot (plot);
  // }

  {
    // Ptr<> lossmodel = CreateObject<PropagationLossModel>();

    Ptr<PropagationLossModel> log_combined = CreateObject<LogDistancePropagationLossModel> ();
    Ptr<PropagationLossModel> fading = CreateObject<RandomPropagationLossModel> ();
    const Ptr<NormalRandomVariable> nrv = CreateObject<NormalRandomVariable> ();
    nrv->SetAttribute ("Mean", DoubleValue (0));
    nrv->SetAttribute ("Variance", DoubleValue (32));
    fading->SetAttribute("Variable", ns3::PointerValue(nrv));
    // fading->AssignStreams(nrv->GetStream());
    log_combined->SetNext(fading);

    Gnuplot plot = TestProbabilistic(log_combined);

    // Gnuplot plot = TestProbabilistic (log_combined);
    plot.SetTitle ("ns3::LogDistancePropagationLossModel");
    gnuplots.AddPlot (plot);
  }

  gnuplots.GenerateOutput (std::cout);

  // produce clean valgrind
  Simulator::Destroy ();
  return 0;
}
