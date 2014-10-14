/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;
import java.io.*;
import java.net.*;
import java.util.*;
import java.util.Vector;

import ciberIF.*;
import java.util.logging.Level;
import java.util.logging.Logger;
/**
 *main class
 * @author vedran
 */
public class IaMRLab1 {
    
    /**
     * @param args the command line arguments
     */      
    public static void main(String[] args) {
        String host, robName;
	int pos; 
	int arg;

	//default values
	host = "localhost";
	robName = "Vedran";
	pos = 1;


        // parse command-line arguments
	try {
	    arg = 0;
	    while (arg<args.length) {
		if(args[arg].equals("-pos")) {
			if(args.length > arg+1) {
				pos = Integer.valueOf(args[arg+1]).intValue();
				arg += 2;
			}
		}
		else if(args[arg].equals("-robname")) {
			if(args.length > arg+1) {
				robName = args[arg+1];
				arg += 2;
			}
		}
		else if(args[arg].equals("-host")) {
			if(args.length > arg+1) {
				host = args[arg+1];
				arg += 2;
			}
		}
		else throw new Exception();
	    }
	}
	catch (Exception e) {
		print_usage();
		return;
	}
        
        //create a new Agent
        Agent myAgent = AgentFactory.CreateNewAgent(robName, pos, host);
        
        myAgent.Init();
        myAgent.Start();
        
    }
    
    static void print_usage() {
             System.out.println("Usage: java jClient [-robname <robname>] [-pos <pos>] [-host <hostname>[:<port>]]");
    }
    
}
