package com.github.rinde.rinsim.examples.experiment;

import java.util.List;
import com.github.rinde.rinsim.core.model.comm.MessageContents;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;

public class Mssgs implements MessageContents {
	
	Optional<String> sendingMessage ;
	Optional<List<List<Point>>> routingInfo ;
	Optional<List<Point>> senderRoute ;
	Optional<Long> timeStamp;
	Optional<List<String>> aGV_IDs ;
	Optional<String> senderID;
	Optional<Double> eucDistanceToTarget;
	
	public Mssgs(){
		sendingMessage = Optional.absent();
		routingInfo = Optional.absent();
		senderRoute = Optional.absent();
		timeStamp = Optional.absent();
		aGV_IDs = Optional.absent();
		senderID = Optional.absent();
		eucDistanceToTarget = Optional.absent();
	}
	
	public void setTimeStamp(long timeS){
		timeStamp = Optional.of(timeS);	
	}
	
	public void setEucDistance(double dist){
		eucDistanceToTarget = Optional.of(dist);	
	}
	
	public void handshake(String Msg){
		sendingMessage = Optional.of(Msg);	
	}
	
	public void setSenderRoute(List<Point> Route){
		senderRoute = Optional.of(Route);
	}
	
	public void setSenderID(String ID){
		senderID = Optional.of(ID);
	}
	
	public Optional<List<Point>> getSenderRoute(){
		Optional<List<Point>> route = senderRoute;
		return route;
	}
	
	public String getSenderID(){
		String ID = senderID.get();
		return ID;
	}
	
	public long getTimeStamp(){
		long timeS = timeStamp.get();
		return timeS;	
	}
	
	public double getEucDistance(){
		double dist = eucDistanceToTarget.get();
		return dist;	
	}
	
	public Optional<String> getHandshake(){
		Optional<String> handshake = sendingMessage;
		return handshake;
	}
	
}