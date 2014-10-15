/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

/**
 *
 * @author vedran
 */
public interface Subject {
    public void Register(Observer obj);
    public void Unregister(Observer obj);
    
    public void NotifyObservers();
    //public Object GetUpdate(Observer obj);
}
