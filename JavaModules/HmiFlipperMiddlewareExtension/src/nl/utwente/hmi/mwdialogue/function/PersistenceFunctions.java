package nl.utwente.hmi.mwdialogue.function;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

import nl.utwente.hmi.mwdialogue.persistence.PersistentRecord;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Item;
import hmi.flipper.informationstate.Record;

public class PersistenceFunctions {
	
	private Record is;

	public PersistenceFunctions(Record is){
		this.is = is;
	}

	/**
	 * Store a certain subsection of the IS for future use.
	 * Duplicate IDs will be overwritten
	 * 
	 * @param params This function expects 2 String arguments: first an ID, used to retrieve this data and secondly a path (WITHOUT THE $-SIGN) to store
	 */
	public void storeRecord(Object... params){
		if(params instanceof String[] && params.length == 2){
			String[] p = (String[])params;
			String id = p[0];
			String path = "$"+p[1];
			
			try {
				if(is.getTypeOfPath(path) == Item.Type.Record){
					Record r = is.getRecord(path);
					PersistentRecord pr = new PersistentRecord(path, r);
					
					FileOutputStream f_out = new FileOutputStream("storage/"+id+".data");
					ObjectOutputStream obj_out = new ObjectOutputStream (f_out);
	
					System.out.println(id);
					System.out.println(path);
					System.out.println(r.toString());
					
					obj_out.writeObject ( pr );
					
					f_out.close();
				}
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * Load a subsection of the IS from storage. This record will be placed back in the IS, overwriting it if applicable
	 * @param params one argument: the ID
	 */
	public void loadRecord(Object... params){
		if(params instanceof String[] && params.length == 1){

			String[] p = (String[])params;
			String id = p[0];
			
			try {
				String fileName = "storage/"+id+".data";
						
				File f = new File(fileName);
				
				//does it exist?
				if(!f.isFile()){
					//this means that we didnt find this record, unfortunately
					DefaultRecord r = new DefaultRecord();
					r.set("loadedsuccess","FALSE");
					is.set("$persistence", r);
				} else {
					FileInputStream f_in = new FileInputStream(f);
					ObjectInputStream obj_in = new ObjectInputStream (f_in);
			
					Object obj = obj_in.readObject();
		
					if (obj instanceof PersistentRecord)
					{
						PersistentRecord pr = (PersistentRecord) obj;
						is.set(pr.getPath(), pr.getRec());
	
						DefaultRecord r = new DefaultRecord();
						r.set("loadedsuccess","TRUE");
						is.set("$persistence", r);
					}
	
					f_in.close();
				}
			} catch (ClassNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				//this means that we didnt find this record, unfortunately
				DefaultRecord r = new DefaultRecord();
				r.set("loadedsuccess","FALSE");
				is.set("$persistence", r);
				e.printStackTrace();
			}

		}
	}
	
}
