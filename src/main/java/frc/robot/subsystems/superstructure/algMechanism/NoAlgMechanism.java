package frc.robot.subsystems.superstructure.algMechanism;

public class NoAlgMechanism implements AlgMechanismIO{

    private double voltage = 0;

    public NoAlgMechanism(){
    }

    @Override
    public void setDesiredExtension(double extension){
        voltage = extension*12;
    }

    /*@Override
    public void setOutputPercentage(double percentage){
    }*/

    @Override
    public void updateInputs(AlgMechanismIOInputs inputs){
        inputs.rackAppliedVolts = voltage;
    }
}