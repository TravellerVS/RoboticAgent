/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package iamr.lab1;

import ciberIF.ciberIF;

/**
 *
 * @author vedran
 */
public class BasicIF {
    private ciberIF _cif;
    private void SetCIF(ciberIF cif){ this._cif = cif; }
    private ciberIF GetCIF(){ return this._cif; }
    
    BasicIF (ciberIF cif)
    {
        this.SetCIF(cif);
    }
    
    public void Init(String robName, int pos, String host)
    {
        this.GetCIF().InitRobot(robName, pos, host);
    }
    
    public void Finish()
    {
        this.GetCIF().Finish();
    }
    public boolean GetFinished()
    {
        return this.GetCIF().GetFinished();
    }
    
    public double GetTime()
    {
        return this.GetCIF().GetTime();
    }
    
}
