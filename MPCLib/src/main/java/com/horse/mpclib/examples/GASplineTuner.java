package com.horse.mpclib.examples;

import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.Obstacle;
import com.horse.mpclib.lib.geometry.Circle2d;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.motion.Spline;
import com.horse.mpclib.lib.motion.SplineGenerator;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

public class GASplineTuner {
    private static final boolean PRINT_SPLINE = false;
    private static final int MAX_GENERATIONS = 10;
    private static final int POPULATION_SIZE = 300;

    private static final double MIN_TUNE_VALUE = -25d;
    private static final double MAX_TUNE_VALUE = 25d;

    private static final int TERMS_TO_TUNE = 4 * 7 + 2; //Must be some multiple of 4

    private static final int elitismCount = 2;
    private static final double crossoverProbability = 0.9d;
    private static final double mutationProbability = 0.1d;

    private static double[][] populationValues;

    private int currentGeneration;

    private boolean importedData;

    public static void main(String... args) {
        System.out.println("Running spline smoothing algorithm with degree " + (2 + getTermsToTune() / 4));
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        ComputerDebugger.init(new RobotGAMPC());
        ComputerDebugger.getRobot().init_debug();
        GASplineTuner tuner = new GASplineTuner();
        tuner.init(false);
        tuner.simulateGenerations(100);
    }

    public static void saveData() {
        if(getPopulationValues() == null) {
            return;
        }

        try {
            BufferedWriter out = new BufferedWriter(new FileWriter("SplineTuningData.dat"));
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
            BufferedReader in = new BufferedReader(new FileReader("SplineTuningData.dat"));
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
        Spline initialSpline = SplineGenerator.getInitialSpline(2 + getTermsToTune() / 4, new Pose2d(9, 9, new Rotation2d(0d, false)),
                new Translation2d(2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 2] + 50d, 2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 1] + 50d), Arrays.copyOfRange(getPopulationValues()[index], 0, getPopulationValues()[index].length - 1 - 3 - (getTermsToTune() - 8 - 2) / 2 - 2));
        Spline finalSpline = SplineGenerator.getTerminatingSpline(2 + getTermsToTune() / 4, initialSpline, new Rotation2d(0d, false),
                new Translation2d(2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 2] + 50d, 2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 1] + 50d),
                new Pose2d(144d - 9d, 144d - 9d, new Rotation2d(0d, false)),
                Arrays.copyOfRange(getPopulationValues()[index], 4 + 1 + (getTermsToTune() - 8 - 2) / 2, getPopulationValues()[index].length - 1 - 2));

        if(isPrintSpline()) {
            System.out.println(initialSpline);
            System.out.println(finalSpline);
        }

        int steps = 500;
        Translation2d[] values = new Translation2d[2 * steps];
        for(int i = 0; i < steps; i++) {
            values[i] = initialSpline.evaluate((double)(i) / steps);
            values[i + steps] = finalSpline.evaluate((double)(i) / steps);
        }

        for(int i = 0; i < values.length - 1; i++) {
            try {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(new Line2d(values[i], values[i + 1])));
            } catch (IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }

        Translation2d obstacle = new Translation2d(144d / 2d, 144d / 2d);
        Obstacle obstacleObject = new Obstacle(obstacle, 9d, 1d);
        double minDistanceToObstacle = Math.min(initialSpline.getMinDistanceFromPoint(obstacle), finalSpline.getMinDistanceFromPoint(obstacle));

        double obstacleCost = 0d;
        if(minDistanceToObstacle < 18d) {
            obstacleCost = 1E9d;
            for(int i = 0; i < getPopulationValues()[index].length - 1; i++) {
                getPopulationValues()[index][i] = getRandomTuneValue();
            }
        }

        try {
            ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(new Circle2d(
                    obstacleObject.getLocation(), obstacleObject.getObstacleRadius() / 0.0254d
            )));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        ComputerDebugger.sendMessage();

        //Update cost value which is set equal to the mean curvature
        getPopulationValues()[index][getPopulationValues()[index].length - 1] = 10d * (Math.pow(initialSpline.getMeanCurvature() + finalSpline.getMeanCurvature(), 2d) +
                Math.pow(initialSpline.getMeanDCurvature() + finalSpline.getMeanDCurvature(), 2d)) + (initialSpline.getArcLength() + finalSpline.getArcLength()) + obstacleCost;
        System.out.print("Iteration: " + index + "\t");
        System.out.print(Arrays.toString(getPopulationValues()[index]));
        System.out.println("\t" + getPopulationValues()[index][getPopulationValues()[index].length - 1]);
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
            Arrays.sort(getPopulationValues(), (o1, o2) -> {
                double comparison = o1[getTermsToTune()] - o2[getTermsToTune()];
                return Double.compare(comparison, 0d);
            });

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

        int index = 0;
        Spline initialSpline = SplineGenerator.getInitialSpline(2 + getTermsToTune() / 4, new Pose2d(9, 9, new Rotation2d(0d, false)),
                new Translation2d(2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 2] + 50d, 2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 1] + 50d), Arrays.copyOfRange(getPopulationValues()[index], 0, getPopulationValues()[index].length - 1 - 3 - (getTermsToTune() - 8 - 2) / 2 - 2));
        Spline finalSpline = SplineGenerator.getTerminatingSpline(2 + getTermsToTune() / 4, initialSpline, new Rotation2d(0d, false),
                new Translation2d(2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 2] + 50d, 2d * getPopulationValues()[index][getPopulationValues()[index].length - 1 - 1] + 50d),
                new Pose2d(144d - 9d, 144d - 9d, new Rotation2d(0d, false)),
                Arrays.copyOfRange(getPopulationValues()[index], 4 + 1 + (getTermsToTune() - 8 - 2) / 2, getPopulationValues()[index].length - 1 - 2));

        int steps = 500;
        Translation2d[] values = new Translation2d[2 * steps];
        for(int i = 0; i < steps; i++) {
            values[i] = initialSpline.evaluate((double)(i) / steps);
            values[i + steps] = finalSpline.evaluate((double)(i) / steps);
        }

        for(int i = 0; i < values.length - 1; i++) {
            try {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(new Line2d(values[i], values[i + 1])));
            } catch (IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }

        ComputerDebugger.sendMessage();

        System.out.println(initialSpline);
        System.out.println(finalSpline);
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
        double totalNonnormalizedProbability = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> Math.pow(totalCost - chromosome[chromosome.length - 1], 2)).sum();
        double[] selectionProbability = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> Math.pow(totalCost - chromosome[chromosome.length - 1], 2) / totalNonnormalizedProbability).toArray();
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
        GASplineTuner.populationValues = populationValues;
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

    public static boolean isPrintSpline() {
        return PRINT_SPLINE;
    }
}
