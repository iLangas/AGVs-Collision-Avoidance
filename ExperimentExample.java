package com.github.rinde.rinsim.examples.experiment;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.collect.Lists.newArrayList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import javax.measure.unit.SI;

import org.apache.commons.math3.random.MersenneTwister;
import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.SimulatorAPI;
import com.github.rinde.rinsim.core.model.ModelBuilder;
import com.github.rinde.rinsim.core.model.comm.CommModel;
import com.github.rinde.rinsim.core.model.pdp.DefaultPDPModel;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.TimeWindowPolicy.TimeWindowPolicies;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.CollisionGraphRoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModelBuilders;
import com.github.rinde.rinsim.core.model.road.RoadModelBuilders.CollisionGraphRMB;
import com.github.rinde.rinsim.experiment.Experiment;
import com.github.rinde.rinsim.experiment.Experiment.SimulationResult;
import com.github.rinde.rinsim.experiment.ExperimentResults;
import com.github.rinde.rinsim.experiment.MASConfiguration;
import com.github.rinde.rinsim.geom.Graph;
import com.github.rinde.rinsim.geom.LengthData;
import com.github.rinde.rinsim.geom.ListenableGraph;
import com.github.rinde.rinsim.geom.MultimapGraph;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.pdptw.common.AddDepotEvent;
import com.github.rinde.rinsim.pdptw.common.AddParcelEvent;
import com.github.rinde.rinsim.pdptw.common.AddVehicleEvent;
import com.github.rinde.rinsim.pdptw.common.StatsStopConditions;
import com.github.rinde.rinsim.pdptw.common.TimeLinePanel;
import com.github.rinde.rinsim.scenario.Scenario;
import com.github.rinde.rinsim.scenario.StopConditions;
import com.github.rinde.rinsim.scenario.TimeOutEvent;
import com.github.rinde.rinsim.scenario.TimedEvent;
import com.github.rinde.rinsim.scenario.TimedEventHandler;
import com.github.rinde.rinsim.ui.View;
import com.github.rinde.rinsim.ui.renderers.CommRenderer;
import com.github.rinde.rinsim.ui.renderers.GraphRoadModelRenderer;
import com.github.rinde.rinsim.ui.renderers.PDPModelRenderer;
import com.github.rinde.rinsim.ui.renderers.PlaneRoadModelRenderer;
import com.github.rinde.rinsim.ui.renderers.Renderer;
import com.github.rinde.rinsim.ui.renderers.RoadUserRenderer;
import com.github.rinde.rinsim.util.TimeWindow;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;

/**
 * This example shows how to use the {@link Experiment} class to define and run
 * an experiment. It shows how to construct a {@link Scenario} and
 * {@link MASConfiguration} which are both requirements for an example. The
 * intermediate steps in the example are documented, however, make sure to also
 * read the documentation of each method for extra information about what it
 * does.
 * <p>
 * If this class is run on MacOS it might be necessary to use
 * -XstartOnFirstThread as a VM argument.
 * @author Rinde van Lon
 */
public final class ExperimentExample {
  // some constants used in the experiment
  private static final Point RESOLUTION = new Point(600, 600);
  private static final double VEHICLE_SPEED_KMH = 60d;

  private static final long M1 = 60 * 1000L;
  private static final long M4 = 4 * 60 * 1000L;
  private static final long M20 = 20 * 60 * 1000L;
  private static final long M30 = 30 * 60 * 1000L;
  
  private static final long EXP_TIME_DURATION = 10 * 60 * 60 * 1000L;
  /*
  static final int VERTICAL_SPACING = 16;
  static final int HORIZONTAL_SPACING = 16;
  static final int HEIGHT = 160;
  static final int WIDTH = 160;
  */
  static final int VERTICAL_SPACING = 16;
  static final int HORIZONTAL_SPACING = 16;
  static final int HEIGHT = 160;
  static final int WIDTH = 160;
  
  static final double SPACING = 30d;
  static final double MAX_HEIGHT = HEIGHT * SPACING;
  static final double MAX_WIDTH = WIDTH * SPACING;
  static final double MIN_VER = VERTICAL_SPACING * SPACING;
  static final double MIN_HOR = HORIZONTAL_SPACING * SPACING;
  
  static final int VEHICLES_NUMBER = 4 ;
  static final int PARCELS_NUMBER = 10 ;
  static final int DEPOT_NUMBER = 3;
  
  static final long RANDOM_SEED = 53L;
  private static final double VEHICLE_LENGTH = 2d;
  
  static ArrayList<Point> point_dep_list = new ArrayList<Point>(DEPOT_NUMBER);

  private ExperimentExample() {}

  /**
   * It is possible to use the application arguments directly for configuring
   * the experiment. The '-h' or '--help' argument will show the list of
   * options.
   * @param args The arguments supplied to the application.
   */
  public static void main(String[] args) {
    int uiSpeedUp = 1;
    final int index = Arrays.binarySearch(args, "speedup");

    String[] arguments = args;
    if (index >= 0) {
      checkArgument(arguments.length > index + 1,
        "speedup option requires an integer indicating the speedup.");
      uiSpeedUp = Integer.parseInt(arguments[index + 1]);
      checkArgument(uiSpeedUp > 0, "speedup must be a positive integer.");
      final List<String> list = new ArrayList<String>(Arrays.asList(arguments));
      list.remove(index + 1);
      list.remove(index);
      arguments = list.toArray(new String[] {});
    }

    final Optional<ExperimentResults> results;

    // Starts the experiment builder.
    results = Experiment.builder()

      // Adds a configuration to the experiment. A configuration configures an
      // algorithm that is supposed to handle or 'solve' a problem specified by
      // a scenario. A configuration can handle a scenario if it contains an
      // event handler for all events that occur in the scenario. The scenario
      // in this example contains four different events and registers an event
      // handler for each of them.
      .addConfiguration(MASConfiguration.builder()
        .addEventHandler(AddDepotEvent.class, AddDepotEvent.defaultHandler())
        .addEventHandler(AddParcelEvent.class, AddParcelEvent.defaultHandler())
        // There is no default handle for vehicle events, here a non functioning
        // handler is added, it can be changed to add a custom vehicle to the
        // simulator.
        .addEventHandler(AddVehicleEvent.class, CustomVehicleHandler.INSTANCE)
        .addEventHandler(TimeOutEvent.class, TimeOutEvent.ignoreHandler())

        // Note: if you multi-agent system requires the aid of a model (e.g.
        // CommModel) it can be added directly in the configuration. Models that
        // are only used for the solution side should not be added in the
        // scenario as they are not part of the problem.
        .build())

      // Adds the newly constructed scenario to the experiment. Every
      // configuration will be run on every scenario.
      .addScenario(createScenario())

      // The number of repetitions for each simulation. Each repetition will
      // have a unique random seed that is given to the simulator.
      .repeat(1)

      // The master random seed from which all random seeds for the
      // simulations will be drawn.
      .withRandomSeed(0)

      // The number of threads the experiment will use, this allows to run
      // several simulations in parallel. Note that when the GUI is used the
      // number of threads must be set to 1.
      .withThreads(1)

      // We add a post processor to the experiment. A post processor can read
      // the state of the simulator after it has finished. It can be used to
      // gather simulation results. The objects created by the post processor
      // end up in the ExperimentResults object that is returned by the
      // perform(..) method of Experiment.
      .usePostProcessor(new ExamplePostProcessor())
      // Adds the GUI just like it is added to a Simulator object.
      .showGui(View.builder()
    	.with(GraphRoadModelRenderer.builder().withMargin(2))

    	.with(RoadUserRenderer.builder()
    			.withImageAssociation(AgvAgent.class,	"/graphics/flat/forklift2.png"))
        .with(PDPModelRenderer.builder())
        
        .with(CommRenderer.builder()
                .withReliabilityColors()
                .withMessageCount())
        
        .with(TimeLinePanel.builder())
        
        .withResolution((int) RESOLUTION.x, (int) RESOLUTION.y)
        .withAutoPlay()
       // .withAutoClose()
        // For testing we allow to change the speed up via the args.
        .withSpeedUp(uiSpeedUp)
        .withTitleAppendix("Experiments example"))

      // Starts the experiment, but first reads the command-line arguments
      // that are specified for this application. By supplying the '-h' option
      // you can see an overview of the supported options.
      .perform(System.out, arguments);

    if (results.isPresent()) {

      for (final SimulationResult sr : results.get().getResults()) {
        // The SimulationResult contains all information about a specific
        // simulation, the result object is the object created by the post
        // processor, a String in this case.
        System.out.println(
          sr.getSimArgs().getRandomSeed() + " " + sr.getResultObject());
      }
    } else {
      throw new IllegalStateException("Experiment did not complete.");
    }
  }

  /**
   * Defines a simple scenario with one depot, one vehicle and three parcels.
   * Note that a scenario is supposed to only contain problem specific
   * information it should (generally) not make any assumptions about the
   * algorithm(s) that are used to solve the problem.
   * @return A newly constructed scenario.
   */
  

  
  @SuppressWarnings("unchecked")
static Scenario createScenario() {
    // In essence a scenario is just a list of events. The events must implement
    // the TimedEvent interface. You are free to construct any object as a
    // TimedEvent but keep in mind that implementations should be immutable.
	  
	  Point point_dep;
	  
	  for (int i = 0; i <= DEPOT_NUMBER-1; i++){
		  double random_x = Math.random();
		  double random_y = Math.random();
		  final double xcoor = random_x * (MAX_WIDTH + 1) - (random_x * (MAX_WIDTH + 1))%MIN_HOR;
		  final double ycoor = random_y * (MAX_HEIGHT + 1) - (random_y * (MAX_HEIGHT+ 1))%MIN_VER;
		  point_dep = new Point(xcoor,ycoor);
		  point_dep_list.add(point_dep);
	  }
	  
	// Create list of Vehicles, Parcels, and Depots
	// Dropping points of parcels are the Depot points
	  final Iterable<AddVehicleEvent> list_of_vehicles = create_VehicleEvents(VEHICLES_NUMBER);
	  final Iterable<AddParcelEvent> list_of_parcels = create_ParcelEvents(PARCELS_NUMBER);
	  final Iterable<AddDepotEvent> list_of_depots = create_DepotEvents(DEPOT_NUMBER);
	//final ListenableGraph<> gr = new ListenableGraph<>(createGrid(WIDTH, HEIGHT, VERTICAL_SPACING, HORIZONTAL_SPACING, SPACING));
	  
	  
    return Scenario.builder()

      // Adds one depot.
      //.addEvent(AddDepotEvent.create(-1, p1_loc))

      // Adds one vehicle.
      //.addEvent(AddVehicleEvent.create(-1, VehicleDTO.builder()
    	//.startPosition(new Point(480,480))
        //.speed(VEHICLE_SPEED_KMH)
        //.build()))
      
      // Adds randomly a given number of vehicles
      .addEvents(list_of_vehicles)
      
      // Adds parcels 
      .addEvents(list_of_parcels)
      
      // Adds deposit spots
      .addEvents(list_of_depots)

      // Three add parcel events are added. They are announced at different
      // times and have different time windows.
        

      // Signals the end of the scenario. Note that it is possible to stop the
      // simulation before or after this event is dispatched, that depends on
      // the stop condition (see below).
      .addEvent(TimeOutEvent.create(EXP_TIME_DURATION))
      .scenarioLength(EXP_TIME_DURATION)

      // Adds a plane road model as this is part of the problem
      .addModel(RoadModelBuilders.dynamicGraph(new ListenableGraph<>(
    		  createGrid(WIDTH, HEIGHT, VERTICAL_SPACING, HORIZONTAL_SPACING, SPACING))
     		 ).withCollisionAvoidance()
    		  .withDistanceUnit(SI.METER)
              .withVehicleLength(VEHICLE_LENGTH))
      
      .addModel(CommModel.builder())
      
      
      // Adds the pdp model
      .addModel(
        DefaultPDPModel.builder()
          .withTimeWindowPolicy(TimeWindowPolicies.LIBERAL))
      // The stop condition indicates when the simulator should stop the
      // simulation. Typically this is the moment when all tasks are performed.
      // Custom stop conditions can be created by implementing the StopCondition
      // interface.
      .setStopCondition(StopConditions.or(
        StatsStopConditions.timeOutEvent(),
        StatsStopConditions.vehiclesDoneAndBackAtDepot()))
      .build();
  }
  
  static Iterable<AddVehicleEvent> create_VehicleEvents(int integ){
	// Create ArrayList where our Events are stored
	ArrayList<AddVehicleEvent> myVehicleList = new ArrayList<AddVehicleEvent>();
	
	final double intervW = (MAX_WIDTH + 1)/integ;
	final double intervH = (MAX_HEIGHT + 1)/integ;
	
	for (int i = 1; i<=integ; i++){
		double rand_x = Math.random();
		double rand_y = Math.random();
		// make sure that the vehicles are on the grid
		final double xcoor = rand_x * (MAX_WIDTH + 1) - (rand_x * (MAX_WIDTH + 1))%MIN_HOR;
		final double ycoor = intervH*(i) + rand_y * (intervH*i - intervH*(i) + 1) - (intervH*(i) + rand_y * (intervH*i - intervH*(i) + 1))%MIN_VER;
		myVehicleList.add(AddVehicleEvent.create(-1, VehicleDTO.builder()
		    	.startPosition(new Point(ycoor,xcoor))
		        .speed(VEHICLE_SPEED_KMH)
		        .build()));
	}
	
	Iterable<AddVehicleEvent> vehicleBuilder = myVehicleList;
	return vehicleBuilder;
}
  
  static Iterable<AddParcelEvent> create_ParcelEvents(int integ){
		ArrayList<AddParcelEvent> myParcelList = new ArrayList<AddParcelEvent>();
		
		for (int i = 0; i <= integ-1; i++){
			double random_x = Math.random();
			double random_y = Math.random();
			
			final double xcoor_pic = random_x * (MAX_WIDTH + 1) - (random_x * (MAX_WIDTH + 1))%MIN_HOR;
			final double ycoor_pic = random_y * (MAX_HEIGHT + 1) - (random_y * (MAX_HEIGHT+ 1))%MIN_VER;
			
			final Point p_pic = new Point(xcoor_pic,ycoor_pic);	
			
			myParcelList.add(AddParcelEvent.create(Parcel.builder(p_pic, new Point(480,480))
					.neededCapacity(0)
					.orderAnnounceTime(M1)
					.pickupTimeWindow(TimeWindow.create(M1, M20))
					.deliveryTimeWindow(TimeWindow.create(M4, M30))
					.buildDTO()));
		}
		
		Iterable<AddParcelEvent> parcelBuilder = myParcelList;
		return parcelBuilder;
	}
  
  static Iterable<AddDepotEvent> create_DepotEvents(int integ){
		ArrayList<AddDepotEvent> myDepotList = new ArrayList<AddDepotEvent>();
		
		for (int i = 0; i <= integ-1; i++){
			
			myDepotList.add(AddDepotEvent.create(-1, point_dep_list.get(i)));
		}
		
		Iterable<AddDepotEvent> depotBuilder = myDepotList;
		return depotBuilder;
	}

enum CustomVehicleHandler implements TimedEventHandler<AddVehicleEvent> {
    INSTANCE {
      public void handleTimedEvent(AddVehicleEvent event, SimulatorAPI sim) {
    	AgvAgent agv_agent = new AgvAgent(event.getVehicleDTO());
        sim.register(agv_agent);
        agv_agent.getAvgId();
        ;
    	  // add you own vehicle to the simulator here
      }
    }
  }
  
    static Graph<LengthData> createGrid(int width, int height, int hLines,
    	     int vLines, double distance) {
    	   final Graph<LengthData> graph = new MultimapGraph<LengthData>();

    	   int v = 0;
    	   // draw vertical lines
    	   for (int i = 0; i < width + 1; i++) {
    	     Point prev = new Point(i * distance, 0);
    	     if (i % vLines == 0) {
    	       for (int j = 1; j < height; j++) {
    	         final Point cur = new Point(i * distance, j * distance);
//    	         if (v % 2 == 0) {
    	           graph.addConnection(prev, cur);
//    	         } else {
    	           graph.addConnection(cur, prev);
//    	         }
    	         prev = cur;
    	       }
    	       v++;
    	     }
    	   }

    	   int y = 1;
    	   for (int i = 0; i < height; i++) {
    	     Point prev = new Point(0, i * distance);
    	     if (i % hLines == 0) {
    	       for (int j = 1; j < width + 1; j++) {
    	         final Point cur = new Point(j * distance, i * distance);
    	         //if (y % 2 == 0) {
    	           graph.addConnection(prev, cur);
    	         //} else {
    	           graph.addConnection(cur, prev);
    	         //}
    	         prev = cur;
    	       }
    	     }
    	     y++;
    	   }
    	   return graph;
    	 }
}