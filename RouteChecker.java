package com.github.rinde.rinsim.examples.experiment;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.LinkedList;
import java.util.List;

import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;

public class RouteChecker {
	private static final Exception Exception = null;
	// distance between each cross-road
	final double x_crossroadDistance = ExperimentExample.HORIZONTAL_SPACING * ExperimentExample.SPACING;
	final double y_crossroadDistance = ExperimentExample.VERTICAL_SPACING * ExperimentExample.SPACING;
	final double secureTime = ExperimentExample.HORIZONTAL_SPACING;
	final int checkRadious = 6;//1 * ExperimentExample.HORIZONTAL_SPACING;	
	//public List<Point> newRoute = new LinkedList<Point>();
	public boolean collisionDanger = false;
	public Point previousCrossRoad = null;
	public Point bestPoint = null;	
	final double SPACING = ExperimentExample.SPACING;
	
	public RouteChecker(AgvAgent agv, List<Point> currentRoute, List<Point> otherAgvRoute, RoadModel rm) {
		
		// Counters for the while loop
		int i = 0;
		
		while (i < currentRoute.size() && i < checkRadious && !(checkInRange(otherAgvRoute,currentRoute.get(i)))) {
			i++;
		}
		if (collisionDanger){
			System.out.println(agv.getThisAgvID().toString().split("@")[1]);
			System.out.println("Current Position");
			System.out.println(rm.getPosition(agv));
			System.out.println("collitionDanger Point");
			System.out.println(currentRoute.get(i-1));
			
			System.out.println("Previous Crossroad");
			try {
				previousCrossRoad = getPreviousCrossRoad(currentRoute,i-1, agv.destination);
			} catch (bookingException e) {
				e.printStackTrace();
			}
			System.out.println(previousCrossRoad);
			
			try {
				bestPoint = getBestDetour(previousCrossRoad, currentRoute.get(i-1), rm, agv.destination);
			} catch (Exception e) {
				System.out.println(e);
			}
			System.out.println("bestPoint");
			System.out.println(bestPoint);
		}
	}
	
	private Point getBestDetour(Point previosCrossRoad, Point collitionDangerPoint, RoadModel rm, Optional<Point> endPoint) throws bookingException{
		List<Point> twoPoints = getPossiblePoints(previosCrossRoad,collitionDangerPoint,rm);
		List<Double> lengths = new ArrayList<Double>();
		Point bestPoint = null;
		for(int i = 0;i<twoPoints.size(); i++){
			double temp = Math.sqrt(Math.pow(twoPoints.get(i).x-endPoint.get().x,2)+Math.pow(twoPoints.get(i).y-endPoint.get().y,2));
			lengths.add(temp);
		}
		if(lengths.size() >1){
			if(lengths.get(0)>lengths.get(1)){
				bestPoint = twoPoints.get(0);
			}
			else{
				bestPoint = twoPoints.get(1);
			}
		}
		else{
			bestPoint = twoPoints.get(0);
		}
		return bestPoint;
	}

	private List<Point> getPossiblePoints(Point previosCrossRoad, Point collitionDangerPoint,RoadModel rm) throws bookingException {
		double deltaX = collitionDangerPoint.x - previosCrossRoad.x;
		double deltaY = collitionDangerPoint.y - previosCrossRoad.y;
		List<Point> twoPoints = new ArrayList<Point>();
		
		if(isCrossRoad(previosCrossRoad) != true){
			throw new bookingException("No CROSSRAD!!!!");
		}
		
		if (deltaX == 0.0){
			Point newP1 = new Point(previosCrossRoad.x + x_crossroadDistance,previosCrossRoad.y);
			Point newP2 = new Point(previosCrossRoad.x - x_crossroadDistance,previosCrossRoad.y);
			try{
				rm.getShortestPathTo(previosCrossRoad, newP1);
				if(isCrossRoad(newP1) != true){
					throw new bookingException("Point is no CrossRoad.");
				}
				twoPoints.add(newP1);
			} catch(Exception e){}
			try{
				rm.getShortestPathTo(previosCrossRoad, newP2);
				if(isCrossRoad(newP2) != true){
					throw new bookingException("Point is no CrossRoad.");
				}
				twoPoints.add(newP2);
			} catch(Exception e){
				System.out.println(e);
			}
		}
		
		else if (deltaY == 0.0){
			Point newP1 = new Point(previosCrossRoad.x,previosCrossRoad.y + y_crossroadDistance);
			Point newP2 = new Point(previosCrossRoad.x,previosCrossRoad.y - y_crossroadDistance);
			try{
				rm.getShortestPathTo(previosCrossRoad, newP1);
				twoPoints.add(newP1);
			} catch(Exception e){}
			try{
				rm.getShortestPathTo(previosCrossRoad, newP2);
				twoPoints.add(newP2);
			} catch(Exception e){}
		}
		if (deltaX == 0.0 && deltaY == 0.0){
			throw new bookingException("No possible route found:Both 0");
		}
		if (deltaX != 0.0 && deltaY != 0.0){
			throw new bookingException("No possible route found:Both not zero");
		}
		if (twoPoints.isEmpty()){
			throw new bookingException("Its Empty.");
		}
		return twoPoints;
	}

	public Point getPreviousCrossRoad(List<Point> prevRoute, int collitionDangerIndex, Optional<Point> destination) throws bookingException{
		int i = collitionDangerIndex - 1;
		boolean crossRoadFound = false;
		Point previousCrossRoad = null;
		while(i >= 0 && !crossRoadFound){
			if (isCrossRoad(prevRoute.get(i))){
				previousCrossRoad = prevRoute.get(i);
				crossRoadFound = true;
				if(previousCrossRoad.equals(prevRoute.get(collitionDangerIndex))){
					System.out.println("Broblem in first method");
				}
			}
			i = i - 1;
		}
		if (!(prevRoute.size()>1)){
			
/*			System.out.println(prevRoute.size());
			System.out.println(prevRoute.toString());
			System.out.println(destination.get());
*/			throw new bookingException("Previous route is no longer than 1:");
		}

		if (crossRoadFound == false && (prevRoute.size()>1)){
			
			try {
				previousCrossRoad = getOppossiteDirectionCrossroad(prevRoute, collitionDangerIndex);
				if(previousCrossRoad.equals(prevRoute.get(collitionDangerIndex))){
					System.out.println("Broblem in second method");
				}
			} catch (Exception e) {
				throw e;
			}
			
		}

		return previousCrossRoad;
	}
	
	private Point getOppossiteDirectionCrossroad(List<Point> prevRoute, int collitionDangerIndex) throws bookingException{
		Point prevPoint = null;
		Point currentPoint =  null;
		//TODO: Sometimes routes with length 1 show up. Not handled.
		if(collitionDangerIndex - 1 < 0 ){
			prevPoint = prevRoute.get(collitionDangerIndex);
			currentPoint =  prevRoute.get(collitionDangerIndex + 1);
		}
		else{
			prevPoint = prevRoute.get(collitionDangerIndex - 1);
			currentPoint =  prevRoute.get(collitionDangerIndex);
		}
		double deltaX = currentPoint.x-prevPoint.x;
		double deltaY = currentPoint.y-prevPoint.y;
		double normDeltaX = deltaX/Math.abs(deltaX);
		double normDeltaY = deltaY/Math.abs(deltaY);
		boolean crossRoadFound = false;
		
		
		if (deltaX != 0.0){
			if (deltaY != 0.0){
				throw new bookingException("Previous direction cannot be stablished: Two deltas not zero.");
			}				
		}
		
		
		if (deltaX != 0.0){
			int i = 1;
			while(i*SPACING<=x_crossroadDistance + SPACING && crossRoadFound == false){
				Point tempPoint = new Point(-normDeltaX*i*SPACING + currentPoint.x,currentPoint.y);
				if (isCrossRoad(tempPoint)){
					previousCrossRoad = tempPoint;
					crossRoadFound = true;
				}
				i++;
			}
		}
		else if(deltaY != 0.0){
			int i = 1;
			while(i*SPACING<=y_crossroadDistance + SPACING && crossRoadFound == false){
				Point tempPoint = new Point(currentPoint.x,-normDeltaY*i*SPACING + currentPoint.y);
				if (isCrossRoad(tempPoint)){
					previousCrossRoad = tempPoint;
					crossRoadFound = true;
				}
				i++;
			}
		}
		else{
			throw new bookingException("Previous direction cannot be stablished: Two deltas zero.");}
		if(previousCrossRoad == null){
			throw new bookingException("Previous direction cannot be stablished: Point left null in getOppositeDirection");
		}
		
		return previousCrossRoad;
		
	}
	
	public Optional<Point> getBestPoint(){
		Optional<Point> bestP = null;
		bestP = Optional.of(bestPoint);
		return bestP;
	}
	
	public Optional<Point> getPreviousCrossRoad(){
		Optional<Point> bestP = null;
		bestP = Optional.of(previousCrossRoad);
		return bestP;
	}
	
	public boolean getCollisionDanger(){
		return collisionDanger;
	}

	public boolean checkInRange(List<Point> currentRoute, Point otherAgvRoutePoint){
		boolean check = false;
		int k = 1;
		while ( k < checkRadious && k<currentRoute.size()){
			if (currentRoute.get(k).equals(otherAgvRoutePoint)){
				check = true;
				collisionDanger = true;
			}
			k++;
		}
		return check;
	}

	public boolean isCrossRoad(Point point) {
		boolean isCross = (point.x%x_crossroadDistance == 0 && point.y%y_crossroadDistance == 0);
		
		return isCross;
	}

}


class bookingException extends Exception {
	/**
	 * 
	 */
	private static final long serialVersionUID = -5072804062288278385L;

	public bookingException(String msg){
	super(msg);
}
	}