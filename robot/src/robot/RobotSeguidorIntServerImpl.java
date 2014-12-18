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
 * This class is the implemetation object for your IDL interface.
 *
 * Let the Eclipse complete operations code by choosing 'Add unimplemented methods'.
 */
public class RobotSeguidorIntServerImpl extends corba.robot.RobotSeguidorIntPOA {
	
	CamaraInt camara;
	
	private IPYPortD ipyport;
	
	private String _nombre = "nombre";
	private int _id;
	public String _IORrob;
	private PuntosRobotD _puntrob;
	private PosicionD _posObj;
	private int _idLider = -1;
	
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
		_idLider = arg0;
	}

	@Override
	public void ModificarObjetivo(PosicionD arg0) {
		_posObj = arg0;
	}

	@Override
	public void ModificarPosicion(PosicionD arg0) {
		robot.fijarPosicion(arg0);
	}

	@Override
	public void ObtenerEstado(EstadoRobotDHolder arg0) {
		
		EstadoRobotD ERD = new EstadoRobotD(_nombre, _id, _IORrob, null, robot.posicionRobot(), _posObj, _idLider);
		
		arg0.value = ERD;
		
	}
	
	public class robotDifusion extends Thread {
		
		private Braitenberg B;
		private Destino Dest;
		
		public robotDifusion() {
					
		}

		@Override
		public void run() {
			
			B = new Braitenberg();
			Destino dest = new Destino();
			
			_posObj	= new PosicionD(200,40);
			suscripcionD SD = camara.SuscribirRobot(_IORrob);
			
			_id = SD.id;
			ipyport = SD.iport;
			escenarioD = SD.esc;
			escenario = new Escenario(escenarioD);
			robot = new RobotKhepera(new PosicionD(0,0), escenario, 0);
			
			
			
			DifusionMulticast DM = new DifusionMulticast(ipyport);
			
			while(true){
				
				InstantaneaD ID = (InstantaneaD)DM.receiveObject();
				
				// Not following anyone
				if(_idLider != -1) {	
					for(EstadoRobotD ERD : ID.estadorobs) {		
						if(ERD.id == _idLider) {
							_posObj = ERD.puntrob.centro;
						}	
					}	
				}
				
				float[] sensors = robot.leerSensores();
				IzqDer vel = B.calcularVelocidad(sensors);
				Polares polares = robot.posicionPolares();
				Trayectoria tray = new Trayectoria(polares, _posObj);
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