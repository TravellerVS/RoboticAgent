/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;
import ciberIF.*;
/**
 *This Factory class makes different types of Agents 
 * depending on the requested type of agent
 * @author vedran
 */
public class AgentFactory {
    /*
     * Constructor method
     */
    AgentFactory()
    {
        
    }
    public static Agent CreateNewAgent(String robName, int pos, String host) 
    {
        return new BasicAgent(robName, pos, host);
    }
    
}
