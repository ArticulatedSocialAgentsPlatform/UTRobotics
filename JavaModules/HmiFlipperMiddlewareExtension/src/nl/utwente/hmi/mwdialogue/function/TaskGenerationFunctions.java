package nl.utwente.hmi.mwdialogue.function;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import nl.utwente.hmi.mwdialogue.persistence.PersistentRecord;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Item;
import hmi.flipper.informationstate.Record;

public class TaskGenerationFunctions {
	

	private Record is;

	private Map<Integer, ArrayList<Task>> tasks;
	private Task previousTask;
	
	public TaskGenerationFunctions(Record is){
		this.is = is;
		initTasks();
	}

	/**
	 * Hardcoded generation of predefined tasks in various difficulties
	 */
	private void initTasks() {
		tasks = new HashMap<Integer, ArrayList<Task>>();

		// Difficulty 1 - always balanced with same color on same position
		ArrayList<Task> t1 = new ArrayList<Task>();
		t1.add(new Task(1, "0r00r0"));
		t1.add(new Task(1, "y0000y"));
		t1.add(new Task(1, "00gg00"));
		t1.add(new Task(1, "00rr00"));
		t1.add(new Task(1, "0y00y0"));
		t1.add(new Task(1, "g0000g"));
		
		tasks.put(1, t1);

		// Difficulty 2 - always inbalanced with same color on different position
		ArrayList<Task> t2 = new ArrayList<Task>();
		t2.add(new Task(2, "0r000r"));
		t2.add(new Task(2, "y000y0"));
		t2.add(new Task(2, "00g00g"));
		t2.add(new Task(2, "g00g00"));
		t2.add(new Task(2, "0y0y00"));
		t2.add(new Task(2, "00r00r"));
		
		tasks.put(2, t2);

		// Difficulty 3 - always inbalanced with different color on same position
		ArrayList<Task> t3 = new ArrayList<Task>();
		t3.add(new Task(3, "0r00y0"));
		t3.add(new Task(3, "0r00g0"));
		t3.add(new Task(3, "y0000r"));
		t3.add(new Task(3, "g0000r"));
		t3.add(new Task(3, "00ry00"));
		
		tasks.put(3, t3);

		// Difficulty 4 - different colors on different positions, sometimes balance
		ArrayList<Task> t4 = new ArrayList<Task>();
		t4.add(new Task(4, "0r000y"));
		t4.add(new Task(4, "r000y0"));
		t4.add(new Task(4, "00r0y0"));
		t4.add(new Task(4, "y00g00"));
		t4.add(new Task(4, "0r0g00"));
		t4.add(new Task(4, "r000g0"));
		t4.add(new Task(4, "r00g00"));
		
		tasks.put(4, t4);

		// Difficulty 5 - 3 of different colors on different positions, sometimes balance
		ArrayList<Task> t5 = new ArrayList<Task>();
		t5.add(new Task(5, "yy0g00"));
		t5.add(new Task(5, "0yyg00"));
		t5.add(new Task(5, "0yy0r0"));
		t5.add(new Task(5, "r0r0g0"));
		
		tasks.put(5, t5);

		// Difficulty 6 - PUZZLE TASKS 2 of different colors but only 1 position known
		ArrayList<Task> t6 = new ArrayList<Task>();
		t6.add(new Task(6, "y000r0"));
		t6.add(new Task(6, "0r000y"));
		t6.add(new Task(6, "0r0g00"));
		t6.add(new Task(6, "00g0r0"));
		t6.add(new Task(6, "y00g00"));
		t6.add(new Task(6, "00g00y"));
		
		tasks.put(6, t6);

		// Difficulty 7 - PUZZLE TASKS 2 of different colors both positions unknown
		ArrayList<Task> t7 = new ArrayList<Task>();
		t7.add(new Task(7, "y000r0"));
		t7.add(new Task(7, "00g0r0"));
		t7.add(new Task(7, "00g00y"));
		
		tasks.put(7, t7);

		// Difficulty 8 - PUZZLE TASKS 3 of pots both positions unknown
		ArrayList<Task> t8 = new ArrayList<Task>();
		t8.add(new Task(8, "0yy0r0"));
		t8.add(new Task(8, "0yyg00"));
		t8.add(new Task(8, "rr00g0"));
		
		tasks.put(8, t8);
	}

	/**
	 * Load an assignment of the indicated difficulty.
	 * The assignment will be stored in the IS on location $isglobal.task
	 * Following information will be stored:
	 * $isglobal.task.loadedcorrect = "true"
	 * $isglobal.task.text = "assignment text"
	 * $isglobal.task.difficulty = "1"
	 * $isglobal.task.correctpositions = "0y00y0"
	 * $isglobal.task.correctpots = "yy"
	 * $isglobal.task.imagefile = "1_0y00y0"
	 * 
	 * @param params One argument expected: the difficuly (number between 1..8)
	 */
	public void loadTask(Object... params){
		if(params instanceof String[] && params.length == 1){
			String[] p = (String[])params;
			int difficulty = 2;
			try{
				difficulty = Integer.parseInt(p[0]);
			} catch(NumberFormatException nfe){}
			
			if(difficulty > 8){
				difficulty = 8;
			}
			
			if(difficulty < 1){
				difficulty = 1;
			}
			
			ArrayList<Task> ts = tasks.get(difficulty);
			Random random = new Random();
			//get a random one, as long as it isnt the same as our previous assignment
			//TODO: this is now very bruteforce, look for nicer solution (queue or stack..?)
			Task nextTask = ts.get(random.nextInt(ts.size()));
			while(nextTask.equals(previousTask)){
				nextTask = ts.get(random.nextInt(ts.size()));
			}
			
			DefaultRecord r = new DefaultRecord();
			r.set("loadedcorrect","TRUE");
			r.set("text",nextTask.getText());
			r.set("difficulty",difficulty);
			r.set("correctpositions",nextTask.getCorrectPositions());
			r.set("correctpots",nextTask.getCorrectPots());
			r.set("imagefile",nextTask.getImageFile());
			
			is.set("$isglobal.task", r);
			
			previousTask = nextTask;
		}
	}
	
	

	private class Task {

		private String correctPositions;
		private String correctPots;
		private String textRepresentation;
		private String imageFile;
		private int difficulty;
		
		/**
		 * Difficulty determines the way we give the assignment. The correct positions indicate when an assignment is correct, and contain info about which pots to use
		 * @param difficulty the difficulty (1..8) 1-5 indicates normal task, 6 is puzzle task with 1 free pot, 7 is puzzle task with 2 free pots and 8 is puzzle task with 3 free pots  
		 * @param correctPositions a string of 6 characters indicating where the pots should be placed. r y g stand for red, yellow, grey and 0 stands for nothing
		 */
		private Task(int difficulty, String correctPositions){
			this.difficulty = difficulty;
			this.correctPositions = correctPositions;
			generateText();
		}

		public String getCorrectPositions(){
			return correctPositions;
		}
		public String getCorrectPots(){
			return correctPots;
		}
		public String getText(){
			return textRepresentation;
		}
		public String getImageFile(){
			return imageFile;
		}

		/**
		 * here we generate the actual text of the assignment
		 */
		private void generateText() {
			//first strip out all empty positions, this leaves us with an overview of which pots to use
			correctPots = correctPositions.replaceAll("0", "");
			
			//now we generate the textual representations of the pots
			ArrayList<PotPosition> pots = new ArrayList<PotPosition>();
			for(int i = 0; i<correctPositions.length(); i++){
				char c = correctPositions.charAt(i);
				if(c != '0'){
					pots.add(new PotPosition(c, i+1));
				}
			}
			
			//now depending on the difficulty we can generate the full assignment text
			if(difficulty > 0 && difficulty <= 5){
				//are we using 2 or 3 pots..?
				if(pots.size() == 2){
					textRepresentation = String.format("Zet een %s potje op pin %s, en een %s potje op pin %s. Wat gaat er gebeuren met de balans?", 
										pots.get(0).getColor(), pots.get(0).getPosition(), 
										pots.get(1).getColor(), pots.get(1).getPosition());
				} else if(pots.size() == 3){
					textRepresentation = String.format("Zet een %s potje op pin %s, een %s potje op pin %s, en een %s potje op pin %s. Wat gaat er gebeuren met de balans?", 
							pots.get(0).getColor(), pots.get(0).getPosition(), 
							pots.get(1).getColor(), pots.get(1).getPosition(), 
							pots.get(2).getColor(), pots.get(2).getPosition());
				}
			} else if(difficulty == 6 && pots.size() == 2){
				textRepresentation = String.format("Gebruik een %s potje en een %s potje. Zet het %s potje op pin %s. Waar moet je het andere potje neerzetten om de balans in evenwicht te houden?", 
						pots.get(0).getColor(), pots.get(1).getColor(), 
						pots.get(0).getColor(), pots.get(0).getPosition());
			} else if(difficulty == 7 && pots.size() == 2){
				textRepresentation = String.format("Gebruik een %s potje en een %s potje. Waar moet je de potjes neerzetten om de balans in evenwicht te houden?", 
						pots.get(0).getColor(), pots.get(1).getColor());
			} else if(difficulty == 8 && pots.size() == 3){
				textRepresentation = String.format("Gebruik een %s potje, een %s potje en een %s potje. Waar moet je de potjes neerzetten om de balans in evenwicht te houden?", 
						pots.get(0).getColor(), pots.get(1).getColor(), pots.get(2).getColor());
			}
			
			imageFile = "img"+difficulty+correctPositions;

		}
		
		
	}
	
	/**
	 * Stores a textual representation of a pot's color and position
	 * @author davisond
	 *
	 */
	private class PotPosition {
		
		private String color;
		private String position;
		
		public String getPosition() {
			return position;
		}

		public void setPosition(int position) {
			this.position = "";
			
			if(position == 1){
				this.position = "een";
			} else if(position == 2){
				this.position = "twee";
			} else if(position == 3){
				this.position = "drie";
			} else if(position == 4){
				this.position = "vier";
			} else if(position == 5){
				this.position = "vijf";
			} else if(position == 6){
				this.position = "zes";
			}		
		}

		public String getColor() {
			return color;
		}

		public void setColor(char color) {
			this.color = "";

			if(color == 'r'){
				this.color = "rood";
			} else if(color == 'y'){
				this.color = "geel";
			} else if(color == 'g'){
				this.color = "grijs";
			}
		}


		private PotPosition(char color, int position){
			setColor(color);
			setPosition(position);
		}
		
	}
	
}
