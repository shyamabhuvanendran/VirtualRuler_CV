package com.java.Test;

import org.opencv.core.Core;
import com.java.ImageProcess.ReferenceObject;
import com.java.ImageProcess.Stereo;


/**
 * Execute this class to start the application
 * @author shyama
 *
 */
public class ExecuteMe {

	public static void main(String[] args) {
		
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		//Starts Reference Object Method
		ReferenceObject R=new ReferenceObject();
		//R.referenceObjectMethod();
		
		//Starts Stereo Method
		Stereo S =new Stereo();
		S.useStereoMethod();
		
	}
}
