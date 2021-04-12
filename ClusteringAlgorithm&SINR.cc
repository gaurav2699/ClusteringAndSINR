/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * NIST-developed software is provided by NIST as a public
 * service. You may use, copy and distribute copies of the software in
 * any medium, provided that you keep intact this entire notice. You
 * may improve, modify and create derivative works of the software or
 * any portion of the software, and you may copy and distribute such
 * modifications or works. Modified works should carry a notice
 * stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the
 * National Institute of Standards and Technology as the source of the
 * software.
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES
 * NO WARRANTY OF ANY KIND, EXPRESS, IMPLIED, IN FACT OR ARISING BY
 * OPERATION OF LAW, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
 * NON-INFRINGEMENT AND DATA ACCURACY. NIST NEITHER REPRESENTS NOR
 * WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE UNINTERRUPTED
 * OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST DOES NOT
 * WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE
 * SOFTWARE OR THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE
 * CORRECTNESS, ACCURACY, RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 *
 * You are solely responsible for determining the appropriateness of
 * using and distributing the software and you assume all risks
 * associated with its use, including but not limited to the risks and
 * costs of program errors, compliance with applicable laws, damage to
 * or loss of data, programs or equipment, and the unavailability or
 * interruption of operation. This software is not intended to be used
 * in any situation where a failure could cause risk of injury or
 * damage to property. The software developed by NIST employees is not
 * subject to copyright protection within the United States.
 */


#include "ns3/lte-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store.h"
#include "ns3/gnuplot.h"
#include "ns3/phy-stats-calculator.h"
#include <cfloat>
#include <sstream>
#include <vector>
#include <map>
#include <math.h>
#include <string>
#include <fstream>

using namespace ns3;
using std::vector;

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

// This trace will log packet transmissions and receptions from the application
// layer.  The parameter 'localAddrs' is passed to this trace in case the
// address passed by the trace is not set (i.e., is '0.0.0.0' or '::').  The
// trace writes to a file stream provided by the first argument; by default,
// this trace file is 'UePacketTrace.tr'
void
UePacketTrace (Ptr<OutputStreamWrapper> stream, const Address &localAddrs, std::string context, Ptr<const Packet> p, const Address &srcAddrs, const Address &dstAddrs)
{
  std::ostringstream oss;
  *stream->GetStream () << Simulator::Now ().GetNanoSeconds () / (double) 1e9 << "\t" << context << "\t" << p->GetSize () << "\t";
  if (InetSocketAddress::IsMatchingType (srcAddrs))
    {
      oss << InetSocketAddress::ConvertFrom (srcAddrs).GetIpv4 ();
      if (!oss.str ().compare ("0.0.0.0")) //srcAddrs not set
        {
          *stream->GetStream () << Ipv4Address::ConvertFrom (localAddrs) << ":" << InetSocketAddress::ConvertFrom (srcAddrs).GetPort () << "\t" << InetSocketAddress::ConvertFrom (dstAddrs).GetIpv4 () << ":" << InetSocketAddress::ConvertFrom (dstAddrs).GetPort () << std::endl;
        }
      else
        {
          oss.str ("");
          oss << InetSocketAddress::ConvertFrom (dstAddrs).GetIpv4 ();
          if (!oss.str ().compare ("0.0.0.0")) //dstAddrs not set
            {
              *stream->GetStream () << InetSocketAddress::ConvertFrom (srcAddrs).GetIpv4 () << ":" << InetSocketAddress::ConvertFrom (srcAddrs).GetPort () << "\t" <<  Ipv4Address::ConvertFrom (localAddrs) << ":" << InetSocketAddress::ConvertFrom (dstAddrs).GetPort () << std::endl;
            }
          else
            {
              *stream->GetStream () << InetSocketAddress::ConvertFrom (srcAddrs).GetIpv4 () << ":" << InetSocketAddress::ConvertFrom (srcAddrs).GetPort () << "\t" << InetSocketAddress::ConvertFrom (dstAddrs).GetIpv4 () << ":" << InetSocketAddress::ConvertFrom (dstAddrs).GetPort () << std::endl;
            }
        }
    }
  else if (Inet6SocketAddress::IsMatchingType (srcAddrs))
    {
      oss << Inet6SocketAddress::ConvertFrom (srcAddrs).GetIpv6 ();
      if (!oss.str ().compare ("::")) //srcAddrs not set
        {
          *stream->GetStream () << Ipv6Address::ConvertFrom (localAddrs) << ":" << Inet6SocketAddress::ConvertFrom (srcAddrs).GetPort () << "\t" << Inet6SocketAddress::ConvertFrom (dstAddrs).GetIpv6 () << ":" << Inet6SocketAddress::ConvertFrom (dstAddrs).GetPort () << std::endl;
        }
      else
        {
          oss.str ("");
          oss << Inet6SocketAddress::ConvertFrom (dstAddrs).GetIpv6 ();
          if (!oss.str ().compare ("::")) //dstAddrs not set
            {
              *stream->GetStream () << Inet6SocketAddress::ConvertFrom (srcAddrs).GetIpv6 () << ":" << Inet6SocketAddress::ConvertFrom (srcAddrs).GetPort () << "\t" << Ipv6Address::ConvertFrom (localAddrs) << ":" << Inet6SocketAddress::ConvertFrom (dstAddrs).GetPort () << std::endl;
            }
          else
            {
              *stream->GetStream () << Inet6SocketAddress::ConvertFrom (srcAddrs).GetIpv6 () << ":" << Inet6SocketAddress::ConvertFrom (srcAddrs).GetPort () << "\t" << Inet6SocketAddress::ConvertFrom (dstAddrs).GetIpv6 () << ":" << Inet6SocketAddress::ConvertFrom (dstAddrs).GetPort () << std::endl;
            }
        }
    }
  else
    {
      *stream->GetStream () << "Unknown address type!" << std::endl;
    }
}

/*
 * To view topology,
 *
 * Run the following command in terminal after running this file:
 *
 * gnuplot topology.plt
 *
 * The topology will get saved in `topology.png`
 */

NS_LOG_COMPONENT_DEFINE ("LteSlInCovrgCommMode1");

//Stores all information about a Point that's used for clustering
class Point {
public:
    double x, y, z;      //Store x, y, z coordinates
    int ptsCnt, cluster;
    bool cu;             //true if the node is a CU (cellular user)
    Ptr<Node> device;    //Store pointer to the NodeDevice
    double getDis(const Point & ot) {
        return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y));
    }
};

//Clustering algorithm
class DBSCAN {
public:
    int minPts;
    double eps;
    vector<Point> points;
    int size;
    vector<vector<int> > adjPoints;
    vector<bool> visited;
    vector<vector<int> > cluster;
    int clusterIdx;

    DBSCAN(double eps, int minPts, vector<Point> points) {
        this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (int)points.size();
        adjPoints.resize(size);
        this->clusterIdx=-1;
    }

    //Runs the entire algorithm
    void run () {
        checkNearPoints();

        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;

            if(isCoreObject(i)) {
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = NOISE;
            }
        }

        cluster.resize(clusterIdx+2);
        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOISE) {
                cluster[points[i].cluster].push_back(i);
            }
            else{
              cluster[clusterIdx+1].push_back(i);
            }
        }
    }

    //Add all neighbours to a cluster
    void dfs (int now, int c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;

        for(auto&next:adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED && points[next].cluster != NOISE) continue;
            dfs(next, c);
        }
    }

    //Find all neighbours of the core point within the eps radius
    void checkNearPoints() {
        for(int i=0;i<size;i++) {
            for(int j=0;j<size;j++) {
                if(i==j) continue;
                if(points[i].getDis(points[j]) <= eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }

    bool isCoreObject(int idx) {
        return (points[idx].ptsCnt >= minPts);
    }

    vector<vector<int> > getCluster() {
        return cluster;
    }
};

//Generate the Topology source file
void GenerateTopologyPlotFile (NodeContainer enbNode, NodeContainer ueNodes, NodeContainer cuNodes, vector<vector<int>> clusters, vector<Point> pos, std::string fileNameWithNoExtension)
{
  std::string graphicsFileName = fileNameWithNoExtension + ".png";
  std::string gnuplotFileName = fileNameWithNoExtension + ".plt";
  std::string plotTitle = "Topology (Labels = Node IDs)";

  Gnuplot plot (graphicsFileName);
  plot.SetTitle (plotTitle);
  plot.SetTerminal ("png size 1280,1024");
  plot.SetLegend ("X", "Y"); //These are the axis, not the legend
  std::ostringstream plotExtras;
  plotExtras << "set xrange [-" << 1.1 * 50 << ":+" << 1.1 * 50 << "]" << std::endl;
  plotExtras << "set yrange [-" << 1.1 * 50 << ":+" << 1.1 * 50 << "]" << std::endl;
  plotExtras << "set linetype 1 pt 3 ps 2 " << std::endl;
  plotExtras << "set linetype 2 lc rgb \"brown\" pt 4 ps 2" << std::endl;
  plot.AppendExtra (plotExtras.str ());

  //eNB
  Gnuplot2dDataset datasetEnodeB;
  datasetEnodeB.SetTitle ("eNodeB");
  datasetEnodeB.SetStyle (Gnuplot2dDataset::POINTS);

  double x = enbNode.Get (0)->GetObject<MobilityModel> ()->GetPosition ().x;
  double y = enbNode.Get (0)->GetObject<MobilityModel> ()->GetPosition ().y;
  std::ostringstream strForLabel;
  strForLabel << "set label \"" << enbNode.Get (0)->GetId () << "\" at " << x << "," << y << " textcolor rgb \"black\" center front offset 0,1";
  plot.AppendExtra (strForLabel.str ());
  datasetEnodeB.Add (x, y);
  plot.AddDataset (datasetEnodeB);

  //CUs
  Gnuplot2dDataset datasetCUs;
  datasetCUs.SetTitle ("CUs");
  datasetCUs.SetStyle (Gnuplot2dDataset::POINTS);
  for (uint32_t rm = 0; rm < cuNodes.GetN (); rm++)
    {
      double x = cuNodes.Get (rm)->GetObject<MobilityModel> ()->GetPosition ().x;
      double y = cuNodes.Get (rm)->GetObject<MobilityModel> ()->GetPosition ().y;
      std::ostringstream strForLabel;
      strForLabel << "set label \"" << cuNodes.Get (rm)->GetId () << "\" at " << x << "," << y << " textcolor rgb \"black\" center front offset 0,1";
      plot.AppendExtra (strForLabel.str ());
      datasetCUs.Add (x, y);
    }
  plot.AddDataset (datasetCUs);

  //Shards
  uint clusterSize = clusters.size();
  Gnuplot2dDataset nodes[clusterSize];
  for (uint i = 0; i < clusterSize; i++)
    {
      bool shard_contains_cu=false;
        for (uint j = 0; j < clusters[i].size(); j++)
        {
            if(pos[(clusters[i][j])].cu){
              shard_contains_cu=true;
              break;
            }
        }
      std::string title;
      if(shard_contains_cu && clusterSize > 1) {
        title = "Shard ";
        title.append(std::to_string(i+1));
      }
      else {
        title = "Unclustered UEs";
      }

      if(clusters[i].size() != 0) {
        nodes[i].SetTitle (title);
        nodes[i].SetStyle (Gnuplot2dDataset::POINTS);

          for (uint j = 0; j < clusters[i].size(); j++)
          {
              double x = pos[clusters[i][j]].device->GetObject<MobilityModel> ()->GetPosition ().x;
              double y = pos[clusters[i][j]].device->GetObject<MobilityModel> ()->GetPosition ().y;
              std::ostringstream strForLabel;
              strForLabel << "set label \"" << pos[clusters[i][j]].device->GetId () << "\" at " << x << "," << y << " textcolor rgb \"black\" center front offset 0,1";
              plot.AppendExtra (strForLabel.str ());
              nodes[i].Add (x, y);
          }

        plot.AddDataset (nodes[i]);
      }
    }

  std::ofstream plotFile (gnuplotFileName.c_str ());
  plot.GenerateOutput (plotFile);
  plotFile.close ();
}

int main (int argc, char *argv[])
{
  bool enableNsLogs = false;
  bool useIPv6 = false;
  float startTime = 2.4;

  CommandLine cmd;
  cmd.AddValue ("enableNsLogs", "Enable ns-3 logging (debug builds)", enableNsLogs);
  cmd.AddValue ("useIPv6", "Use IPv6 instead of IPv4", useIPv6);
  cmd.Parse (argc, argv);

  // Configure the scheduler
  Config::SetDefault ("ns3::RrSlFfMacScheduler::Itrp", UintegerValue (0));
  //The number of RBs allocated per UE for Sidelink
  Config::SetDefault ("ns3::RrSlFfMacScheduler::SlGrantSize", UintegerValue (5));

  //Set the frequency

  Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue (100));
  Config::SetDefault ("ns3::LteUeNetDevice::DlEarfcn", UintegerValue (100));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue (18100));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (50));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (50));

  // Set error models
  Config::SetDefault ("ns3::LteSpectrumPhy::SlCtrlErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::LteSpectrumPhy::SlDataErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::LteSpectrumPhy::DropRbOnCollisionEnabled", BooleanValue (false));

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();
  // parse again so we can override input file default values via command line
  cmd.Parse (argc, argv);

  if (enableNsLogs)
    {
      LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);

      LogComponentEnable ("LteUeRrc", logLevel);
      LogComponentEnable ("LteUeMac", logLevel);
      LogComponentEnable ("LteSpectrumPhy", logLevel);
      LogComponentEnable ("LteUePhy", logLevel);
      LogComponentEnable ("LteEnbPhy", logLevel);
    }

  //Set the UEs power in dBm
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23.0));
  //Set the eNBs power in dBm
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (30.0));

  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (1.0);

  //Create the helpers
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PhyStatsCalculator> phyStats = CreateObject<PhyStatsCalculator> ();

  //Create and set the EPC helper
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  //Create Sidelink helper and set lteHelper
  Ptr<LteSidelinkHelper> proseHelper = CreateObject<LteSidelinkHelper> ();
  proseHelper->SetLteHelper (lteHelper);

  //Set pathloss model
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::Cost231PropagationLossModel"));

  //Enable Sidelink
  lteHelper->SetAttribute ("UseSidelink", BooleanValue (true));

  //Sidelink Round Robin scheduler
  lteHelper->SetSchedulerType ("ns3::RrSlFfMacScheduler");

  //Create eNb Node
  NodeContainer enbNode;
  enbNode.Create (1);

  //Create UE Node
  NodeContainer ueNodes;
  ueNodes.Create (20);

  //Create CU Node
  NodeContainer cuNodes;
  cuNodes.Create (2);

  //Position of the nodes
  vector<Point> pos;

  //eNb
  Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
  positionAllocEnb->Add (Vector (0.0, 0.0, 30.0));

  //Position the UEs
  double positions[20][3] = {{20.0, 7.0, 1.5},
                             {21.0, 10.0, 1.5},
                             {-30.0, 20.0, 1.5},
                             {20.0, 1.0, 1.5},
                             {-20.0, -1.0, 1.5},
                             {15.0, 4.0, 1.5},
                             {-15.0, -4.0, 1.5},
                             {20.0, 30.0, 1.5},
                             {-20.0, -40.0, 1.5},
                             {-13.0, -13.0, 1.5},
                             {13.0, 13.0, 1.5},
                             {21.0, 22.0, 1.5},
                             {15.0, -15.0, 1.5},
                             {-22.0, 0.0, 1.5},
                             {-21.0, -22.0, 1.5},
                             {-22.0, -25.0, 1.5},
                             {21.0, -15.0, 1.5},
                             {-19.0, -5.0, 1.5},
                             {22.0, 7.0, 1.5},
                             {-13.0, 11.0, 1.5}
                            };


  //UEs
  Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator> ();

  //Assign positions to UEs
  for (int i=0; i < 20; i++) {
    positionAllocUe->Add (Vector (positions[i][0], positions[i][1], positions[i][2]));
    pos.push_back({positions[i][0], positions[i][1], positions[i][2], 0, NOT_CLASSIFIED, false, ueNodes.Get (i)});
  }

  //CUs
  Ptr<ListPositionAllocator> positionAllocCu = CreateObject<ListPositionAllocator> ();

  //Assign positions to CUs
  positionAllocCu->Add (Vector (15.0, 0.0, 1.5));
  pos.push_back({15.0 ,0.0, 1.5, 0, NOT_CLASSIFIED, true, cuNodes.Get (0)});
  positionAllocCu->Add (Vector (-15.0, 0.0, 1.5));
  pos.push_back({-15.0 ,0.0, 1.5, 0, NOT_CLASSIFIED, true, cuNodes.Get (1)});

  //Install mobility
  MobilityHelper mobilityeNodeB;
  mobilityeNodeB.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityeNodeB.SetPositionAllocator (positionAllocEnb);
  mobilityeNodeB.Install (enbNode);

  MobilityHelper mobilityUe;
  mobilityUe.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUe.SetPositionAllocator (positionAllocUe);
  mobilityUe.Install (ueNodes);

  MobilityHelper mobilityCu;
  mobilityCu.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityCu.SetPositionAllocator (positionAllocCu);
  mobilityCu.Install (cuNodes);

  //Declare a variable to store clusters
  vector<vector<int> > clusters;
  vector<int> indices;
  for(uint p=0; p<22; p++) {
    indices.push_back(p);
  }
  clusters.push_back(indices);

  //Display Topology before Sharding
  GenerateTopologyPlotFile (enbNode, ueNodes, cuNodes, clusters, pos, "topologyInitial");

  //Install LTE devices to the nodes and fix the random number stream
  int64_t randomStream = 1;
  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice (enbNode);
  randomStream += lteHelper->AssignStreams (enbDevs, randomStream);
  NetDeviceContainer ueDevs = lteHelper->InstallUeDevice (ueNodes);
  randomStream += lteHelper->AssignStreams (ueDevs, randomStream);
  NetDeviceContainer cuDevs = lteHelper->InstallUeDevice (cuNodes);
  randomStream += lteHelper->AssignStreams (cuDevs, randomStream);

  //Configure Sidelink Pool
  Ptr<LteSlEnbRrc> enbSidelinkConfiguration = CreateObject<LteSlEnbRrc> ();
  enbSidelinkConfiguration->SetSlEnabled (true);

  //Preconfigure pool for the group
  LteRrcSap::SlCommTxResourcesSetup pool;

  pool.setup = LteRrcSap::SlCommTxResourcesSetup::SCHEDULED;
  //BSR timers
  pool.scheduled.macMainConfig.periodicBsrTimer.period = LteRrcSap::PeriodicBsrTimer::sf16;
  pool.scheduled.macMainConfig.retxBsrTimer.period = LteRrcSap::RetxBsrTimer::sf640;
  //MCS
  pool.scheduled.haveMcs = true;
  pool.scheduled.mcs = 16;
  //resource pool
  LteSlResourcePoolFactory pfactory;
  pfactory.SetHaveUeSelectedResourceConfig (false); //since we want eNB to schedule

  //Control
  pfactory.SetControlPeriod ("sf40");
  pfactory.SetControlBitmap (0x00000000FF); //8 subframes for PSCCH
  pfactory.SetControlOffset (0);
  pfactory.SetControlPrbNum (22);
  pfactory.SetControlPrbStart (0);
  pfactory.SetControlPrbEnd (49);

  //Data: The ns3::RrSlFfMacScheduler is responsible to handle the parameters

  pool.scheduled.commTxConfig = pfactory.CreatePool ();

  uint32_t groupL2Address = 255;

  enbSidelinkConfiguration->AddPreconfiguredDedicatedPool (groupL2Address, pool);
  lteHelper->InstallSidelinkConfiguration (enbDevs, enbSidelinkConfiguration);

  //pre-configuration for the UEs
  Ptr<LteSlUeRrc> ueSidelinkConfiguration = CreateObject<LteSlUeRrc> ();
  ueSidelinkConfiguration->SetSlEnabled (true);
  LteRrcSap::SlPreconfiguration preconfiguration;
  ueSidelinkConfiguration->SetSlPreconfiguration (preconfiguration);
  lteHelper->InstallSidelinkConfiguration (ueDevs, ueSidelinkConfiguration);

  InternetStackHelper internet;
  internet.Install (ueNodes);
  internet.Install (cuNodes);
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination
  Ipv6Address groupAddress6 ("ff0e::1");     //use multicast address as destination
  Address remoteAddress;
  Address localAddress;
  Ptr<LteSlTft> tft;
  if (!useIPv6)
    {
      // set the default gateway for the UE
      Ipv4StaticRoutingHelper ipv4RoutingHelper;

      //Install the IP stack on the UEs and assign IP address
      Ipv4InterfaceContainer ueIpIface;
      ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

      for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
        {
          Ptr<Node> ueNode = ueNodes.Get (u);
          // Set the default gateway for the UE
          Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }

      //Install the IP stack on the CUs and assign IP address
      Ipv4InterfaceContainer ueIpIfaceCU;
      ueIpIfaceCU = epcHelper->AssignUeIpv4Address (NetDeviceContainer (cuDevs));

      for (uint32_t u = 0; u < cuNodes.GetN (); ++u)
        {
          Ptr<Node> cuNode = cuNodes.Get (u);
          // Set the default gateway for the UE
          Ptr<Ipv4StaticRouting> cuStaticRouting = ipv4RoutingHelper.GetStaticRouting (cuNode->GetObject<Ipv4> ());
          cuStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }


      remoteAddress = InetSocketAddress (groupAddress4, 8000);
      localAddress = InetSocketAddress (Ipv4Address::GetAny (), 8000);
      tft = Create<LteSlTft> (LteSlTft::BIDIRECTIONAL, groupAddress4, groupL2Address);
    }
  else
    {
      // set the default gateway for the UE
      Ipv6StaticRoutingHelper Ipv6RoutingHelper;

      //Install the IP stack on the UEs and assign IP address
      Ipv6InterfaceContainer ueIpIfaceUE;
      ueIpIfaceUE = epcHelper->AssignUeIpv6Address (NetDeviceContainer (ueDevs));

      for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
        {
          Ptr<Node> ueNode = ueNodes.Get (u);
          // Set the default gateway for the UE
          Ptr<Ipv6StaticRouting> ueStaticRouting = Ipv6RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv6> ());
          ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress6 (), 1);
        }

      //Install the IP stack on the CUs and assign IP address
      Ipv6InterfaceContainer ueIpIfaceCU;
      ueIpIfaceCU = epcHelper->AssignUeIpv6Address (NetDeviceContainer (cuDevs));

      for (uint32_t u = 0; u < cuNodes.GetN (); ++u)
        {
          Ptr<Node> cuNode = cuNodes.Get (u);
          // Set the default gateway for the CU
          Ptr<Ipv6StaticRouting> cuStaticRouting = Ipv6RoutingHelper.GetStaticRouting (cuNode->GetObject<Ipv6> ());
          cuStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress6 (), 1);
        }

      remoteAddress = Inet6SocketAddress (groupAddress6, 8000);
      localAddress = Inet6SocketAddress (Ipv6Address::GetAny (), 8000);
      tft = Create<LteSlTft> (LteSlTft::BIDIRECTIONAL, groupAddress6, groupL2Address);
    }
  //Attach each UE to the best available eNB
  lteHelper->Attach (ueDevs);
  lteHelper->Attach (cuDevs);

  //PointToPoint network between the CU nodes
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("16kb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10)));
  NetDeviceContainer internetDevices = p2ph.Install (cuNodes.Get (0), cuNodes.Get (1));

  //Clustering algorithm
  DBSCAN dbScan(15, 2, pos);
  dbScan.run();
  clusters = dbScan.getCluster();

  //Store IP Addresses of every node, one by one
  Ipv4Address ipv4proto;

  for (uint i = 0; i < clusters.size(); i++)
    {
        bool shard_contains_cu=false;
        for (uint j = 0; j < clusters[i].size(); j++)
        {
          if(pos[(clusters[i][j])].cu){
            shard_contains_cu=true;
            break;
          }
        }
        if(shard_contains_cu){
          std::cout << "Shard no: " << i+1 << "\n\n";

            for (uint j = 0; j < clusters[i].size(); j++)
            {
                ipv4proto = pos[clusters[i][j]].device->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
                if(pos[clusters[i][j]].cu) {
                  std::cout << "\tCU  Node ->" << "\tID: " << pos[clusters[i][j]].device->GetId () << "\tIP Address: " << ipv4proto << std::endl;
                }
                else {
                  std::cout << "\tD2D Node ->" << "\tID: " << pos[clusters[i][j]].device->GetId () << "\tIP Address: " << ipv4proto << std::endl;
                }
            }
          std::cout << "\n\n";
        }
        else if(clusters[i].size() != 0){
          std::cout<<"Unclustured nodes: "<<"\n\n";
          for (uint j = 0; j < clusters[i].size(); j++)
          {
              ipv4proto = pos[clusters[i][j]].device->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
              if(pos[clusters[i][j]].cu) {
                std::cout << "\tCU  Node ->" << "\tID: " << pos[clusters[i][j]].device->GetId () << "\tIP Address: " << ipv4proto << std::endl;
              }
              else {
                std::cout << "\tD2D Node ->" << "\tID: " << pos[clusters[i][j]].device->GetId () << "\tIP Address: " << ipv4proto << std::endl;
              }
          }

        std::cout << "\n\n";
        }
    }


  //Display Topology after Sharding
  GenerateTopologyPlotFile (enbNode, ueNodes, cuNodes, clusters, pos, "topologyFinal");

  //Assume first shard for SINR
  vector<Point> selectedShard;
  for(uint i=0; i < clusters[0].size(); i++) {
    if(!pos[clusters[0][i]].cu) {
      selectedShard.push_back(pos[clusters[0][i]]);
    }
  }

  ///*** Configure applications ***///

  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  serverApps.Start (Seconds (startTime));       //Start all server apps at the same time

  //Destination Nodes ISMI values for SINR
  vector<int> destinationNodes;

  proseHelper->ActivateSidelinkBearer (slBearersActivationTime, ueDevs, tft);

  std::cout << "\t======\n" << std::endl;

  //Loop to start clients at regular inetrvals of time
  for(uint i=0; i < selectedShard.size()/2; i++) {
    uint t = selectedShard.size()-i-1;          //Destination is the server

    //Set Application in the UEs
    OnOffHelper sidelinkClient ("ns3::UdpSocketFactory",
                    InetSocketAddress (selectedShard[t].device->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal (), 8000));  //Set destination to be the server
    PacketSinkHelper sidelinkSink ("ns3::UdpSocketFactory", localAddress);
    sidelinkClient.SetConstantRate (DataRate ("16kb/s"), 200);

    //Set Client App
    clientApps = sidelinkClient.Install (selectedShard[i].device);
    clientApps.Start (Seconds (startTime+i));

    //Set Server App
    serverApps = sidelinkSink.Install (selectedShard[t].device);

    //Create a separate trace file for each D2D pair
    AsciiTraceHelper ascii;
    std::string traceFileName = "UePacketTrace";
    std::string extensionName = ".tr";
    traceFileName.append(std::to_string(i+1));
    traceFileName.append(extensionName);

    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream (traceFileName);

    //Trace file table header
    *stream->GetStream () << "time(sec)\ttx/rx\tNodeID\tIMSI\tPktSize(bytes)\tIP[src]\tIP[dst]" << std::endl;

    std::ostringstream oss;

    if (!useIPv6)
      {
        // Set Tx traces
        for (uint16_t ac = 0; ac < clientApps.GetN (); ac++)
          {
            Ipv4Address localAddrs =  clientApps.Get (ac)->GetNode ()->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
            std::cout << "Tx address: " << localAddrs << "\tIMSI: " << clientApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi () << " \tID: " << clientApps.Get (ac)->GetNode ()->GetId () << std::endl;
            oss << "tx\t" << clientApps.Get (ac)->GetNode ()->GetId () << "\t" << clientApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi ();
            clientApps.Get (ac)->TraceConnect ("TxWithAddresses", oss.str (), MakeBoundCallback (&UePacketTrace, stream, localAddrs));
            oss.str ("");
          }

        // Set Rx traces
        for (uint16_t ac = 0; ac < serverApps.GetN (); ac++)
          {
            Ipv4Address localAddrs =  serverApps.Get (ac)->GetNode ()->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
            std::cout << "Rx address: " << localAddrs << "\tIMSI: " << serverApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi () << " \tID: " << serverApps.Get (ac)->GetNode ()->GetId () << std::endl;
            oss << "rx\t" << serverApps.Get (ac)->GetNode ()->GetId () << "\t" << serverApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi ();
            serverApps.Get (ac)->TraceConnect ("RxWithAddresses", oss.str (), MakeBoundCallback (&UePacketTrace, stream, localAddrs));
            destinationNodes.push_back(serverApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi ());
            oss.str ("");
          }
      }
    else
      {
        // Set Tx traces
        for (uint16_t ac = 0; ac < clientApps.GetN (); ac++)
          {
            clientApps.Get (ac)->GetNode ()->GetObject<Ipv6L3Protocol> ()->AddMulticastAddress (groupAddress6);
            Ipv6Address localAddrs =  clientApps.Get (ac)->GetNode ()->GetObject<Ipv6L3Protocol> ()->GetAddress (1,1).GetAddress ();
            std::cout << "Tx address: " << localAddrs << "\tIMSI: " << clientApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi () << " \tID: " << clientApps.Get (ac)->GetNode ()->GetId () << std::endl;
            oss << "tx\t" << clientApps.Get (ac)->GetNode ()->GetId () << "\t" << clientApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi ();
            clientApps.Get (ac)->TraceConnect ("TxWithAddresses", oss.str (), MakeBoundCallback (&UePacketTrace, stream, localAddrs));
            oss.str ("");
          }

        // Set Rx traces
        for (uint16_t ac = 0; ac < serverApps.GetN (); ac++)
          {
            serverApps.Get (ac)->GetNode ()->GetObject<Ipv6L3Protocol> ()->AddMulticastAddress (groupAddress6);
            Ipv6Address localAddrs =  serverApps.Get (ac)->GetNode ()->GetObject<Ipv6L3Protocol> ()->GetAddress (1,1).GetAddress ();
            std::cout << "Rx address: " << localAddrs << "\tIMSI: " << serverApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi () << " \tID: " << serverApps.Get (ac)->GetNode ()->GetId () << std::endl;
            oss << "rx\t" << serverApps.Get (ac)->GetNode ()->GetId () << "\t" << serverApps.Get (ac)->GetNode ()->GetDevice (0)->GetObject<LteUeNetDevice> ()->GetImsi ();
            serverApps.Get (ac)->TraceConnect ("RxWithAddresses", oss.str (), MakeBoundCallback (&UePacketTrace, stream, localAddrs));
            oss.str ("");
          }
      }

    std::cout << std::endl;
    std::cout << "\t======\n" << std::endl;
  }

  ///*** End of application configuration ***///

  NS_LOG_INFO ("Enabling Sidelink traces...");
    lteHelper->EnableSidelinkTraces ();
    lteHelper->EnablePhyTraces();
    lteHelper->EnableRlcTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnablePdcpTraces();
  NS_LOG_INFO ("Starting simulation...");

  Simulator::Stop (Seconds (startTime + 6.0));
  std::cout << "Running Simulation...\n" << std::endl;
    Simulator::Run ();
    Simulator::Destroy ();
  std::cout << "Generating SINR values...\n" << std::endl;

  //Read and analyse the generated SINR text files
  std::ifstream infile;
  double a,e,f,g;
  int b,c,d;
  vector<double> time,rsrp,sinr;
  vector<int> Id,IMSI,RNTI,Cid;

  infile.open("DlRsrpSinrStats.txt");

  if (!infile) {
        std::cout << "Unable to open file" << std::endl;
        exit(1);
  }

  std::string dummyLine;

  getline(infile, dummyLine);

  //Read column wise output from the text file
  while(infile) {
    infile >> a >> b >> c >> d >> e >> f >> g;
    time.push_back(a);
    Id.push_back(b);
    IMSI.push_back(c);
    RNTI.push_back(d);
    rsrp.push_back(e);
    sinr.push_back(f);
    Cid.push_back(g);
  }

  vector<double> sinrOutput;
  for(uint j = 0;j < destinationNodes.size();j++) {
    for(uint i = 0; i < 23;i++){
      if(destinationNodes[j] == IMSI[i]){
        sinrOutput.push_back(10*std::log10(sinr[i]));
      }
    }
  }

  //Display the output in terminal
  std::cout << "\t======================================================\n" << std::endl;
  for(uint i = 0; i <sinrOutput.size();i++){
    for(uint j = 0; j <= i; j++){
      std::cout << "( " << selectedShard[j].device->GetId () << " <--> " << selectedShard[selectedShard.size() - j -1].device->GetId () << " )\t";
    }
    std::cout << std::endl;
    for(uint j = 0; j <= i; j++){
      std::cout << "  " << sinrOutput[j] << "dB  \t";
    }
    std::cout << "\n\n\t======================================================\n" << std::endl;
  }

  infile.close();

  std::cout << "Done...\n" << std::endl;

  return 0;

}
