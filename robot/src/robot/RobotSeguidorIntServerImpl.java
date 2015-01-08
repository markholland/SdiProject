package robot;

import comm.DifusionMulticast;

import khepera.control.Braitenberg;
import khepera.control.Destino;
import khepera.control.Trayectoria;
import khepera.escenario.Escenario;
import khepera.robot.IzqDer;
import khepera.robot.Polares;
import khepera.robot.RobotKhepera;
import corba.camara.CamaraInt;
import corba.camara.IPYPortD;
import corba.camara.suscripcionD;
import corba.instantanea.EstadoRobotD;
import corba.instantanea.EstadoRobotDHolder;
import corba.instantanea.InstantaneaD;
import corba.instantanea.PuntosRobotD;
import corba.khepera.escenario.EscenarioD;
import corba.khepera.robot.PosicionD;
import corba.robot.RobotSeguidorInt;

/**
 * This class is the implementation object for your IDL interface.
 *
 * Let the Eclipse complete operations code by choosing 'Add unimplemented methods'.
 */
public class RobotSeguidorIntServerImpl extends corba.robot.RobotSeguidorIntPOA {
	
	CamaraInt camara;
	
	private IPYPortD ipyport;
	
	private String _nombre = "nombre";
	private int robotId;
	public String IORrob;
	private PosicionD posicionObjetivo;
	private int idLider = -1;
	
	private RobotKhepera robot;
	
	private Escenario escenario;
	private EscenarioD escenarioD;
	
	/**
	 * Constructor for RobotSeguidorIntServerImpl 
	 */
	public RobotSeguidorIntServerImpl() {
	}

	@Override
	public void ModificarEscenario(EscenarioD arg0) {
		escenarioD = arg0;
		escenario = new Escenario(escenarioD);
		robot = new RobotKhepera(new PosicionD(0, 0), escenario, 0);
	}

	@Override
	public void ModificarLider(int arg0) {
		idLider = arg0;
	}

	@Override
	public void ModificarObjetivo(PosicionD arg0) {
		posicionObjetivo = arg0;
	}

	@Override
	public void ModificarPosicion(PosicionD arg0) {
		robot.fijarPosicion(arg0);
	}

	@Override
	public void ObtenerEstado(EstadoRobotDHolder arg0) {
		
		EstadoRobotD estadoRobot = new EstadoRobotD(_nombre, robotId, IORrob, null, robot.posicionRobot(), posicionObjetivo, idLider);
		
		arg0.value = estadoRobot;
		
	}
	
	public class robotDifusion extends Thread {
		
		private Braitenberg brat;
		
		public robotDifusion() {
					
		}

		@Override
		public void run() {
			
			brat = new Braitenberg();
			Destino dest = new Destino();
			
			posicionObjetivo	= new PosicionD(200, 40);
			suscripcionD subscription = camara.SuscribirRobot(IORrob);
			
			robotId = subscription.id;
			ipyport = subscription.iport;
			escenarioD = subscription.esc;
			escenario = new Escenario(escenarioD);
			robot = new RobotKhepera(new PosicionD(0,0), escenario, 0);
					
			DifusionMulticast difusion = new DifusionMulticast(ipyport);
			
			while(true){
				
				InstantaneaD ID = (InstantaneaD)difusion.receiveObject();
				
				// Following someone
				if(idLider != -1) {	
					for(EstadoRobotD ERD : ID.estadorobs) {		
						if(ERD.id == idLider) {
							posicionObjetivo = ERD.puntrob.centro;
						}
					}	
				}
				
				float[] sensors = robot.leerSensores();
				IzqDer vel = brat.calcularVelocidad(sensors);
				Polares polares = robot.posicionPolares();
				Trayectoria tray = new Trayectoria(polares, posicionObjetivo);
				IzqDer vel2 = dest.calcularVelocidad(tray);
				vel2.izq += vel.izq/90;
				vel2.der += vel.der/90;
				
				robot.fijarVelocidad(vel2.izq, vel2.der);
				robot.avanzar();
				
			}		
		}	
	}

	public void start() {
		robotDifusion rD = new robotDifusion();
		rD.start();
	}
}
