package frc.robot.Subsystems.NetworkTableSystems;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.DuplicateFormatFlagsException;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
 * !Warning! This Subsystem is currently in VERY early development, 
 * Please use Smartdashboard Instead!
 */

public class ServerPublishSystem extends SubsystemBase{
    private static final Exception DuplicateTopicNameException = null;
    private static final Exception UnsupportedSupplierTypeException = null;
    public static ServerPublishSystem m_Instance;
    private static NetworkTableInstance m_tableInst = NetworkTableInstance.getDefault();
    private static NetworkTable m_table;
    public class PubGroup<T>{
        GenericPublisher publisher;
        Supplier<T> supplier;
        PubGroup(String topicName, String typeName, Supplier<T> _supp){     //The typeName can be found in file "NetworkTableType.class"
            supplier = _supp;
            publisher = m_table.getTopic(topicName).genericPublish(typeName);
        }
    }
    private List<String> VariableDeclarations = new ArrayList<>();
    private StringArrayPublisher m_variableDeclarationPublisher;

    private List<String> ExceptionHandler = new ArrayList<>();
    private StringArrayPublisher m_ExceptionHandlerPublisher;
    
    private List<PubGroup<Double>> m_doublePublishers = new ArrayList<>();
    private List<PubGroup<Boolean>> m_booleanPublishers = new ArrayList<>();
    private List<PubGroup<Integer>> m_integerPublishers = new ArrayList<>();
    private List<PubGroup<String>> m_stringPublishers = new ArrayList<>();
    public static ServerPublishSystem getInstance(){
        return m_Instance==null?new ServerPublishSystem():m_Instance;
    }
    public ServerPublishSystem(){
        m_tableInst.startServer();
        m_table=m_tableInst.getTable(getName());
        m_variableDeclarationPublisher = m_table.getStringArrayTopic("VariableDeclarations").publish();
    }

    public void putDouble(String _topicName, Supplier<Double> _supp) {
        VariableDeclarations.add(_topicName);
        VariableDeclarations.add("double");
        m_doublePublishers.add(new PubGroup<Double>(_topicName, "double", _supp));
    }
    public void putBoolean(String _topicName, Supplier<Boolean> _supp) {
        VariableDeclarations.add(_topicName);
        VariableDeclarations.add("boolean");
        m_booleanPublishers.add(new PubGroup<Boolean>(_topicName, "boolean", _supp));
    }
    public void putString(String _topicName, Supplier<String> _supp) {
        VariableDeclarations.add(_topicName);
        VariableDeclarations.add("string");
        m_stringPublishers.add(new PubGroup<String>(_topicName, "string", _supp));
    }
    public void putInteger(String _topicName, Supplier<Integer> _supp){
        VariableDeclarations.add(_topicName);
        VariableDeclarations.add("int");
        m_integerPublishers.add(new PubGroup<Integer>(_topicName, "int", _supp));
    }
    public void put(String _topicName, Supplier _supp) { try {
        var tmp = _supp.get();
        if(tmp.getClass() == double.class){
            putDouble(_topicName, _supp);
        }
        else if(tmp.getClass() == boolean.class){
            putBoolean(_topicName, _supp);
        }
        else if(tmp.getClass() == int.class){
            putInteger(_topicName, _supp);
        }
        else if(tmp.getClass() == String.class){
            putString(_topicName, _supp);
        }
        else{
            throw UnsupportedSupplierTypeException;
        }
        } catch (Exception e){
            String ExceptionDescription = "";
            ExceptionDescription += "UnsupportedSupplierTypeException at topic: ";
            ExceptionDescription += _topicName;
            ExceptionHandler.add(ExceptionDescription);
        }
    }
    public void update(){
        m_variableDeclarationPublisher.set((String[]) VariableDeclarations.toArray());
        m_ExceptionHandlerPublisher.set((String[]) ExceptionHandler.toArray());
        for(PubGroup<Double> pub:m_doublePublishers){
            pub.publisher.setDouble(pub.supplier.get());
        }
        for(PubGroup<Boolean> pub:m_booleanPublishers){
            pub.publisher.setBoolean(pub.supplier.get());
        }
        for(PubGroup<String> pub:m_stringPublishers){
            pub.publisher.setString(pub.supplier.get());
        }
        for(PubGroup<Integer> pub:m_integerPublishers){
            pub.publisher.setInteger(pub.supplier.get());
        }
    }
}
