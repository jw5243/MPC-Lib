package com.horse.mpclib.examples;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.MecanumRunnableMPC;
import com.horse.mpclib.lib.util.Time;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

public class GAMPCTuner {
    private static final Time timeout = new Time(60d, TimeUnits.SECONDS);

    private static final int MAX_GENERATIONS = 10;
    private static final int POPULATION_SIZE = 30;

    private static final double MIN_TUNE_VALUE = 0d;
    private static final double MAX_TUNE_VALUE = 200d;

    private static final int TERMS_TO_TUNE = 10; //6 for state cost + 1 for iteration count + /*1 for timestep count*/ + 3 obstacles //4 for input cost

    private static final int elitismCount = 2;
    private static final double crossoverProbability = 0.9d;
    private static final double mutationProbability = 0.1d;

    private static double[][] populationValues;

    private int currentGeneration;

    private boolean importedData;

    public static void main(String... args) {
        GAMPCTuner tuner = new GAMPCTuner();
        ComputerDebugger.init(new RobotGAMPC());
        tuner.init(true);
        tuner.simulateGenerations(2);
    }

    public static void saveData() {
        if(getPopulationValues() == null) {
            return;
        }

        try {
            BufferedWriter out = new BufferedWriter(new FileWriter("TuningData.dat"));
            StringBuilder stringBuilder = new StringBuilder();
            for(int i = 0; i < getPopulationValues().length; i++) {
                for(int j = 0; j < getPopulationValues()[i].length; j++) {
                    stringBuilder.append(getPopulationValues()[i][j]).append(",");
                }

                out.write(stringBuilder.append('\n').toString());
                stringBuilder = new StringBuilder();
            }

            out.close();
            System.out.println("-------------------------------- Successfully saved data --------------------------------");
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    public static void importData() {
        try {
            BufferedReader in = new BufferedReader(new FileReader("TuningData.dat"));
            AtomicInteger runningIndex = new AtomicInteger(0);
            in.lines().forEach((line) -> {
                String[] chromosomeRawData = line.split(",");
                Double[] chromosomeData = Arrays.stream(chromosomeRawData).map(Double::parseDouble).toArray(Double[]::new);
                for(int i = 0; i < chromosomeData.length; i++) {
                    getPopulationValues()[runningIndex.get()][i] = chromosomeData[i];
                }

                runningIndex.incrementAndGet();
            });

            in.close();
        } catch(IOException e) {
            e.printStackTrace();
        }

        System.out.println("-------------------------------- Importing previous data --------------------------------");
        for(int i = 0; i < getPopulationValues().length; i++) {
            Arrays.stream(getPopulationValues()[i]).forEach((value) -> System.out.print(value + ", "));
            System.out.println();
        }
    }

    public void init(boolean importData) {
        setPopulationValues(new double[getPopulationSize()][getTermsToTune() + 1]); //Additional value for storing the cost
        setCurrentGeneration(1);
        setImportedData(importData);
        if(importData) {
            importData();
        }
    }

    public void runIteration(int index) {
        MecanumRunnableMPC.setStateCost(SimpleMatrix.diag(Arrays.copyOfRange(getPopulationValues()[index], 0, getPopulationValues()[index].length - 1 - 4 /*- 4*/)));
        MecanumRunnableMPC.setMaxIterations((int)((20d / getMaxTuneValue()) * getPopulationValues()[index][6]) + 1);
        //MecanumRunnableMPC.setInputCost(SimpleMatrix.diag(Arrays.copyOfRange(getPopulationValues()[index], 6, getPopulationValues()[index].length - 1)).scale(1d / getMaxTuneValue()));

        RobotGAMPC robot = new RobotGAMPC();
        ComputerDebugger.setRobot(robot);
        robot.init_debug();
        for(int i = 0; i < Robot.getObstacles().size(); i++) {
            Robot.getObstacles().get(i).setCostFactor((300d / 200d) * getPopulationValues()[index][7 + i]);
        }

        ComputerDebugger.send(MessageOption.CLEAR_LOG_POINTS);
        ComputerDebugger.send(MessageOption.CLEAR_MOTION_PROFILE);
        ComputerDebugger.sendMessage();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.start_debug();
        boolean failed = false;
        boolean failedLess = false;
        while(!robot.isDone() && TimeUtil.getCurrentRuntime().compareTo(getTimeout()) < 0) {
            try {
                if(robot.getFieldPosition().getTranslation().distance(Robot.getInitialPose().getTranslation()) <  1E-6 &&
                        TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) > 2d) {
                    failed = true;
                    break;
                }

                if(robot.getFieldPosition().getTranslation().distance(robot.getPoseCheck().getTranslation()) < 1E-1 &&
                        (TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) - robot.getPoseCheckTime().getTimeValue(TimeUnits.SECONDS) > 4d)) {
                    failedLess = true;
                    break;
                }

                robot.loop_debug();

                ComputerDebugger.send(MessageOption.ROBOT_LOCATION);
                ComputerDebugger.send(MessageOption.LOG_POINT.setSendValue(robot.getFieldPosition().getTranslation()));
                ComputerDebugger.sendMessage();
                Thread.sleep(10);
            } catch (InterruptedException | IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }

        double distanceAwayFromGoal = robot.getFieldPosition().getTranslation().distance(robot.getPositions().get(0).getTranslation());
        double elapsedTime = (robot.getRuntime() == 0d || failed || failedLess) ? getTimeout().getTimeValue(TimeUnits.SECONDS) : robot.getRuntime();
        int remainingSetpoints = robot.getPositions().size() - 1;
        int timesHittingObstacles = robot.getTimesHittingObstacle();
        double closestDistanceToObstacle = robot.getClosestDistanceToObstacle();
        double angularOffset = Math.abs(robot.getFieldPosition().getRotation().rotateBy(robot.getPositions().get(0).getRotation().inverse()).getDegrees());
        timesHittingObstacles = Math.min(13 * 3, timesHittingObstacles);

        if(failed || (failedLess && robot.getSetpointCount() - 1 <= remainingSetpoints)) {
            timesHittingObstacles = 13 * 3 + 1;
            for(int i = 0; i < getPopulationValues()[index].length - 1; i++) {
                getPopulationValues()[index][i] = getRandomTuneValue();
            }
        } else if(elapsedTime != getTimeout().getTimeValue(TimeUnits.SECONDS)) {
            distanceAwayFromGoal = 0d; //The robot has reached the final position
        } else {
            timesHittingObstacles = 13 * 3;
        }

        double normalizedDistanceCost = 0.01d;
        double normalizedTimeCost = 50d;
        double normalizedRemainingSetpointsCost = 50d;
        double normalizedHittingObstacleCost = 50;
        double normalizedClosestDistanceToObstacleCost = 200d;
        double normalizedAngularOffsetCost = 100d;

        if(Double.isNaN(distanceAwayFromGoal)) {
            distanceAwayFromGoal = normalizedDistanceCost * 144d;
        }

        //Update cost value which is set equal to the time elapsed for the iteration
        getPopulationValues()[index][getPopulationValues()[index].length - 1] =
                normalizedDistanceCost * ((double)(remainingSetpoints) / robot.getSetpointCount()) * (distanceAwayFromGoal / 144d) +
                        normalizedTimeCost * Math.pow(elapsedTime / getTimeout().getTimeValue(TimeUnits.SECONDS), 1d) +
                        normalizedRemainingSetpointsCost * Math.pow((double)(remainingSetpoints) / robot.getSetpointCount(), 2d) +
                        normalizedHittingObstacleCost * (144d / (closestDistanceToObstacle + (144d / 4d))) * Math.pow(timesHittingObstacles / (13 * 3d), 1d) +
                        normalizedAngularOffsetCost * (angularOffset / 5d);
        System.out.print("Iteration: " + index + "\t");
        System.out.print(Arrays.toString(getPopulationValues()[index]));
        System.out.println("\t Took " + elapsedTime + " seconds to finish and hit obstacles " + timesHittingObstacles + " times");
    }

    public void simulateGeneration() {
        System.out.println("-------------------------------- Generation " + getCurrentGeneration() + " --------------------------------");
        if(getCurrentGeneration() == 1 && !isImportedData()) {
            //Take initial generation to be random values
            for(int i = 0; i < getPopulationValues().length; i++) {
                for(int j = 0; j < getPopulationValues()[i].length - 1; j++) {
                    getPopulationValues()[i][j] = getRandomTuneValue();
                }
            }
        } else {
            int k = getElitismCount();
            Arrays.sort(getPopulationValues(), (o1, o2) -> (int)(o1[getTermsToTune()] - o2[getTermsToTune()]));

            double[][] nextPopulationValues = new double[getPopulationSize()][getTermsToTune() + 1];
            for(int i = 0; i < getElitismCount(); i++) {
                System.arraycopy(getPopulationValues()[i], 0, nextPopulationValues[i], 0, nextPopulationValues[0].length);
            }

            while(k < getPopulationSize()) {
                double generationTransitionType = Math.random();
                if(generationTransitionType <= getCrossoverProbability() && k < getPopulationValues().length - 2) {
                    int index1 = getRandomChromosomeIndex();
                    int index2 = getRandomChromosomeIndex();
                    while(index2 == index1) {
                        index2 = getRandomChromosomeIndex();
                    }

                    crossover(nextPopulationValues, k, index1, index2);
                    //System.out.println("Crossing over " + index1 + " with " + index2 + "\tReplacing index " + k + " and " + (k + 1));
                    k++;
                } else {//if(generationTransitionType <= getCrossoverProbability() + getMutationProbability()) {
                    int index = getRandomChromosomeIndex();
                    mutation(nextPopulationValues, k, index);
                    //System.out.println("Mutating " + index + "\tReplacing index " + k);
                }

                k++;
            }

            setPopulationValues(nextPopulationValues);
            //System.out.println("//////////////////////////////////////////////////////////////////////////////////");
        }

        for(int i = 0; i < getPopulationValues().length; i++) {
            runIteration(i);
        }

        System.out.println("Average cost: " + Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> chromosome[chromosome.length - 1]).average().getAsDouble());
        saveData();
        setCurrentGeneration(getCurrentGeneration() + 1);
    }

    public void simulateGenerations(int generations) {
        for(int i = 0; i < generations; i++) {
            simulateGeneration();
        }
    }

    public void crossover(double[][] nextPopulationValues, int nextPopulationIndex, int index1, int index2) {
        int crossoverIndex = getRandomGeneIndex();
        double crossoverValue = Math.random();
        nextPopulationValues[nextPopulationIndex] = getPopulationValues()[index1];
        nextPopulationValues[nextPopulationIndex + 1] = getPopulationValues()[index2];
        nextPopulationValues[nextPopulationIndex][crossoverIndex] = crossoverValue * getPopulationValues()[index1][crossoverIndex] + (1d - crossoverValue) * getPopulationValues()[index2][crossoverIndex];
        nextPopulationValues[nextPopulationIndex + 1][crossoverIndex] = (1d - crossoverValue) * getPopulationValues()[index1][crossoverIndex] + crossoverValue * getPopulationValues()[index2][crossoverIndex];
    }

    public void mutation(double[][] nextPopulationValues, int nextPopulationIndex, int index) {
        int mutatedTuneIndex = getRandomGeneIndex(); //Not terms-to-tune plus one since cost should not be mutated
        double mutatedValue = getRandomTuneValue();
        nextPopulationValues[nextPopulationIndex] = getPopulationValues()[index];
        nextPopulationValues[nextPopulationIndex][mutatedTuneIndex] = mutatedValue;
    }

    private double getRandomTuneValue() {
        return (getMaxTuneValue() - getMinTuneValue()) * Math.random() + getMinTuneValue();
    }

    private int getRandomChromosomeIndex() {
        if(getCurrentGeneration() == 1) {
            return (int)(getTermsToTune() * Math.random());
        }

        double totalCost = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> chromosome[chromosome.length - 1]).sum();
        double totalNonnormalizedProbability = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> Math.pow(totalCost - chromosome[chromosome.length - 1], 4)).sum();
        double[] selectionProbability = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> Math.pow(totalCost - chromosome[chromosome.length - 1], 4) / totalNonnormalizedProbability).toArray();
        double randomValue = Math.random();
        double currentProbabilitySum = 0d;
        for(int i = 0; i < selectionProbability.length; i++) {
            if(randomValue < selectionProbability[i] + currentProbabilitySum && randomValue >= currentProbabilitySum) {
                return i;
            }

            currentProbabilitySum += selectionProbability[i];
        }

        return selectionProbability.length - 1;
    }

    private int getRandomGeneIndex() {
        return (int)(getTermsToTune() * Math.random());
    }

    public static int getMaxGenerations() {
        return MAX_GENERATIONS;
    }

    public static int getPopulationSize() {
        return POPULATION_SIZE;
    }

    public static int getTermsToTune() {
        return TERMS_TO_TUNE;
    }

    public static double[][] getPopulationValues() {
        return populationValues;
    }

    public static void setPopulationValues(double[][] populationValues) {
        GAMPCTuner.populationValues = populationValues;
    }

    public static double getMinTuneValue() {
        return MIN_TUNE_VALUE;
    }

    public static double getMaxTuneValue() {
        return MAX_TUNE_VALUE;
    }

    public static double getCrossoverProbability() {
        return crossoverProbability;
    }

    public static double getMutationProbability() {
        return mutationProbability;
    }

    public static int getElitismCount() {
        return elitismCount;
    }

    public static Time getTimeout() {
        return timeout;
    }

    public int getCurrentGeneration() {
        return currentGeneration;
    }

    public void setCurrentGeneration(int currentGeneration) {
        this.currentGeneration = currentGeneration;
    }

    public boolean isImportedData() {
        return importedData;
    }

    public void setImportedData(boolean importedData) {
        this.importedData = importedData;
    }
}
