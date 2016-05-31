package com.github.rinde.rinsim.examples.experiment;

import java.util.LinkedList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.model.Model.AbstractModel;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.comm.Message;
import com.github.rinde.rinsim.core.model.pdp.PDPModel;
import com.github.rinde.rinsim.core.model.pdp.PDPObject;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.GraphRoadModel;
import com.github.rinde.rinsim.core.model.road.MovingRoadUser;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModels;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.comm.MessageContents;
import com.github.rinde.rinsim.geom.ConnectionData;
import com.github.rinde.rinsim.geom.Graph;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.pdptw.common.AddVehicleEvent;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;
import com.github.rinde.rinsim.examples.experiment.Mssgs;


public class AgvAgent extends Vehicle implements MovingRoadUser, CommUser {

	static final double MIN_RANGE = .2;
	static final double MAX_RANGE = 1.5;
	static final long LONELINESS_THRESHOLD = 10 * 1000;
	
	Optional<RoadModel> roadModel;
	Optional<CommDevice> device;
	Optional<Point> destination;
	Optional<Point> safePositionDestination;
	Optional<Point> normalOperationDestination;
	long lastReceiveTime;
	private final double range;
	private final double reliability;
	private final double rng;
	
	private boolean collisionDanger = false;
	
	private static final double SPEED = 1000d;
	private Optional<Parcel> curr;
	private long timeLeft = 0;
	public static List<String> AGV_id = new LinkedList<String>();
	
	//List<List<Point>> route = new List<List<Point>>(3);
	
	//LinkedList<List<Point>> routingInfo = new LinkedList<List<Point>>(3);  
	
	// the path to follow and the path of another agv sent to this one
	public Optional<List<Point>> pathToFollow;
    public Optional<List<Point>> otherAgvRoute;
	
	// Optional checking of route for collision with another agv
	private Optional<List<Point>> checkedRoute;
	
	// LinkedList that will act as queue for our followPath() method
	private LinkedList<Point> queue_checkedRoute = new LinkedList<>();
	
	public AgvAgent(VehicleDTO dto){
		super(dto);
		curr = Optional.absent();
		checkedRoute = Optional.absent();
		pathToFollow = Optional.absent();
		otherAgvRoute = Optional.absent();
		
		rng = 1000;
	    device = Optional.absent();
	    //roadModel = Optional.absent();
	    destination = Optional.absent();
	    normalOperationDestination = Optional.absent();
	    safePositionDestination = Optional.absent();

	    range = MIN_RANGE + rng * (MAX_RANGE - MIN_RANGE);
	    reliability = 0.9;
	}
	

	@Override
	public void setCommDevice(CommDeviceBuilder builder) {
	    if (range >= 0) {
	        builder.setMaxRange(range);
	      }
	      device = Optional.of(builder
	        .setReliability(reliability)
	        .build());
	}
	
	public AgvAgent getThisAgvID() {
		final RoadModel rm = getRoadModel();
		AgvAgent thisAgv = null;
		//--------Give IDs to all AGVs------
	    final List<AgvAgent> existingAgvs = new LinkedList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    //AGV_id = existingAgvs;
	    for (int i=0; i<rm.getObjectsOfType(AgvAgent.class).size(); i++){
	    	if (i == rm.getObjectsOfType(AgvAgent.class).size() - 1) {
	    		AGV_id.add(existingAgvs.get(i).toString().split("@")[1]);
	    	}
	    }

	    for (int i = 0; i < rm.getObjectsOfType(AgvAgent.class).size(); i++) {
	    	if (rm.getPosition(existingAgvs.get(i)).equals(this.getPosition().get())) {
	    		thisAgv = existingAgvs.get(i);
	    	}
	    }
		return thisAgv;
	}
	
	public void getAvgId(){
		final RoadModel rm = getRoadModel();
		//--------Give IDs to all AGVs------
	    final List<AgvAgent> existingAgvs = new LinkedList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    //AGV_id = existingAgvs;
	    for (int i=0; i<rm.getObjectsOfType(AgvAgent.class).size(); i++){
	    	if (i == rm.getObjectsOfType(AgvAgent.class).size() - 1) {
	    		AGV_id.add(existingAgvs.get(i).toString().split("@")[1]);
	    	}
	    }
	    System.out.println(AGV_id);
	}
	
	
	@Override
	protected void tickImpl(TimeLapse time) {
		final RoadModel rm = getRoadModel();
	    final PDPModel pm = getPDPModel();
	    
//	    final List<AgvAgent> existingAgvs = new LinkedList<AgvAgent>(rm.getObjectsOfType(AgvAgent.class));
	    
	    //--------Package Handling---------- 
	   
	    if (!time.hasTimeLeft()) {
	        return;
	    }
	    if (!curr.isPresent()) {
	      curr = Optional.fromNullable(RoadModels.findClosestObject(
	        rm.getPosition(this), rm, Parcel.class));
	    }
	      
	    if (curr.isPresent() && !collisionDanger) {
	        final boolean inCargo = pm.containerContains(this, curr.get());
	        // sanity check: if it is not in our cargo AND it is also not on the
	        // RoadModel, we cannot go to curr anymore.
	        timeLeft = curr.get().getDeliveryTimeWindow().end();
	        if (!inCargo && !rm.containsObject(curr.get())) {
	          curr = Optional.absent();
	        } 
	        else if (inCargo) {
		          // if it is in cargo, go to its destination
		        destination = Optional.of(curr.get().getDeliveryLocation());
		        
		        	
	    		// Just casually move to the designated location
	      		//rm.moveTo(this, curr.get().getDeliveryLocation(), time);
	      		// save the new destination of the agv
	      		//endPoint = Optional.of(curr.get().getDeliveryLocation());
		  	      		
		        	
	            if (rm.getPosition(this).equals(curr.get().getDeliveryLocation())) {
	            // deliver when we arrive
		             pm.deliver(this, curr.get(), time);
		        }
            } 
	        else {

	            //rm.moveTo(this, curr.get(), time);
	        	// make sure that destination has no value
	        	//destination = Optional.absent();
	        	destination = Optional.of(curr.get().getPickupLocation());
	        	// save the destination of the AGV until it picks the package
	        	//endPoint = Optional.of(curr.get().getPickupLocation());
	
		        if (rm.equalPosition(this, curr.get())) {
		            // pickup package
		        	pm.pickup(this, curr.get(), time);
		        }
            }
	        pathToFollow = Optional.of(rm.getShortestPathTo(this, destination.get()));
	        rm.moveTo(this, destination.get(), time); 
	    }
	    else if(collisionDanger){
	    	pathToFollow = Optional.of(rm.getShortestPathTo(this, destination.get()));
    		System.out.println("Path to follow:");
    		System.out.println(pathToFollow);
	    	rm.moveTo(this, destination.get(), time);
	    	if (rm.getPosition(this).equals(destination.get())) {
	    		pathToFollow = Optional.of(rm.getShortestPathTo(this, destination.get()));
	    		destination = safePositionDestination;
	    		if (rm.getPosition(this).equals(safePositionDestination.get())){
		    		collisionDanger = false;
		    		destination = normalOperationDestination;
	    		}
	        }
	    }

	      
	    //--------Message Handling----------
	    
		if (device.get().getUnreadCount() > 0){
	    	lastReceiveTime = time.getStartTime();
		    
		    // device updates its information about the other AGVs
		    ImmutableList<Message> M = device.get().getUnreadMessages();
	    	
		    for (int i = 0; i< M.size(); i++){
		    	Mssgs temp = (Mssgs) M.get(i).getContents();
		    	
		    	otherAgvRoute = temp.getSenderRoute();
		    	double tempDist = temp.getEucDistance();
		    	
		    	//System.out.println(getThisAgvID().toString().split("@")[1]);
		    	//&& tempDist<eucDistancetoTarg()
		    	if (otherAgvRoute.isPresent() && pathToFollow.isPresent()  && !collisionDanger) {
		    		RouteChecker check = new RouteChecker(this, pathToFollow.get(), otherAgvRoute.get(), rm);
			    	collisionDanger = check.getCollisionDanger();
			    	if(collisionDanger){
			    		normalOperationDestination = destination;
//			    		pathToFollow = Optional.of(rm.getShortestPathTo(this, destination.get()));
			    		destination = check.getPreviousCrossRoad();
					    safePositionDestination = check.getBestPoint();
			    		System.out.println("Changed Destination:");
			    		System.out.println(destination);
			    	}
		    	}

		    }
		    
	    }
		else if (device.get().getUnreadCount() == 0) {
			//Create new Mssgs instanse for sending or broadcasting
			Mssgs newMsg = new Mssgs();		
    		// device broadcasts the route that it plans to follow
			if (destination.isPresent()) {
				newMsg.setSenderID(this.toString().split("@")[1]);
    			newMsg.setSenderRoute(pathToFollow.get());
    			newMsg.setEucDistance(eucDistancetoTarg());
    			device.get().broadcast(newMsg);
			}
    		
    	} 
/*		else if (time.getStartTime() - lastReceiveTime > LONELINESS_THRESHOLD) {
			device.get().broadcast(Messages.WHERE_IS_EVERYBODY);
		}	*/      
	      	
	}
	
/*	public boolean myTimeIsShorter(long myTime, long hisTime){
		boolean itIs = false;
		if (myTime<hisTime){
			itIs = true;
		}
		else if (myTime==hisTime){
			
		}
		return itIs;	
	}
	*/
	public double eucDistancetoTarg(){
		final PDPModel pm = getPDPModel();
		double eDist = 0.0;
		if(curr.isPresent() && pm.containerContains(this, curr.get())){
			eDist = Math.sqrt(Math.pow(curr.get().getDeliveryLocation().x-getPosition().get().x,2)+Math.pow(curr.get().getDeliveryLocation().y-getPosition().get().y,2));
		}
		else if(curr.isPresent() && !pm.containerContains(this, curr.get())){
			eDist = Math.sqrt(Math.pow(curr.get().getPickupLocation().x-getPosition().get().x,2)+Math.pow(curr.get().getPickupLocation().y-getPosition().get().y,2));
		}
		return eDist;	
	}
	
	@Override
	public Optional<Point> getPosition() {
		final RoadModel rm = getRoadModel();
		if (rm.containsObject(this)) {
		    return Optional.of(rm.getPosition(this));
		  }
    return Optional.absent();
	}


}