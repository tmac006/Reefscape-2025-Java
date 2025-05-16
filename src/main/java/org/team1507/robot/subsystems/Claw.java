package org.team1507.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team1507.lib.util.Mutable;
import org.team1507.lib.util.Tunable;
import org.team1507.lib.util.Tunable.TunableDouble;
import org.team1507.lib.util.command.CommandBuilder;
import org.team1507.lib.util.command.GRRSubsystem;
import org.team1507.lib.util.vendors.PhoenixUtil;
import org.team1507.robot.Constants.RioCAN;
import org.team1507.robot.util.ReefSelection;

@Logged
public final class Claw extends GRRSubsystem {


    private static enum ClawSpeed {
        INTAKE(-1),
        OUTTAKE(1),
        CREEP_BACKWARDS(-0.2);       
    

    private TunableDouble speeds;

    private ClawSpeed(double speeds) {
        this.speeds = Tunable.doubleValue("ClawSpeed/speeds/" + name(), speeds);
    }

    private double speeds() {
        return speeds.value();
    }
 }
}