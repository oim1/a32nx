// Local imports
import { Common, FlapConf, AccelFactorMode } from './common';
import { EngineModel } from './EngineModel';
import { FlightModel } from './FlightModel';

export interface StepResults {
    pathAngle: number,
    verticalSpeed: number,
    distanceTraveled: number,
    fuelBurned: number,
    timeElapsed: number,
}

export class Predictions {
    /**
     * Placeholder
     * @param initialAltitude altitude at beginning of step, in feet
     * @param stepSize the size of the altitude step, in feet
     * @param econCAS airspeed during climb (taking SPD LIM & restrictions into account)
     * @param econMach mach during climb, after passing crossover altitude
     * @param climbN1 N1% at CLB setting
     * @param zeroFuelWeight zero fuel weight of the aircraft (from INIT B)
     * @param initialFuelWeight weight of fuel at the end of last step
     * @param headwindAtMidStepAlt headwind component (in knots) at initialAltitude + (stepSize / 2); tailwind is negative
     * @param isaDev ISA deviation (in celsius)
     */
    static altitudeStep(
        initialAltitude: number,
        stepSize: number,
        econCAS: number,
        econMach: number,
        climbN1: number,
        zeroFuelWeight: number,
        initialFuelWeight: number,
        headwindAtMidStepAlt: number,
        isaDev: number,
        tropoAltitude: number,
    ): StepResults {
        const midStepAltitude = initialAltitude + (stepSize / 2);
        const theta = Common.getTheta(midStepAltitude, isaDev);
        const delta = Common.getDelta(theta);
        let mach = Common.CAStoMach(econCAS, delta);

        let eas;
        let tas;
        let usingMach = false;
        // If above crossover altitude, use econMach
        if (mach > econMach) {
            mach = econMach;
            eas = Common.machToEAS(mach, delta);
            tas = Common.machToTAS(mach, theta);
            usingMach = true;
        } else {
            eas = Common.CAStoEAS(econCAS, delta);
            tas = Common.CAStoTAS(econCAS, theta, delta);
        }

        // Engine model calculations
        const theta2 = Common.getTheta2(theta, mach);
        const delta2 = Common.getDelta2(delta, mach);
        const correctedN1 = EngineModel.getCorrectedN1(climbN1, theta2);
        const correctedThrust = EngineModel.tableInterpolation(EngineModel.table1506, correctedN1, mach) * 2 * EngineModel.maxThrust;
        const correctedFuelFlow = EngineModel.getCorrectedFuelFlow(correctedN1, mach, midStepAltitude) * 2;
        const thrust = EngineModel.getUncorrectedThrust(correctedThrust, delta2); // in lbf
        const fuelFlow = EngineModel.getUncorrectedFuelFlow(correctedFuelFlow, delta2, theta2); // in lbs/hour

        const weightEstimate = zeroFuelWeight + initialFuelWeight;

        let pathAngle;
        let verticalSpeed;
        let stepTime;
        let distanceTraveled;
        let fuelBurned;
        let lift = weightEstimate;
        let midStepWeight = weightEstimate;
        let previousMidStepWeight = midStepWeight;
        let iterations = 0;
        do {
            // Assume lift force is equal to weight as an initial approximation
            const liftCoefficient = FlightModel.getLiftCoefficientFromEAS(lift, eas);
            const dragCoefficient = FlightModel.getDragCoefficient(liftCoefficient);
            const accelFactorMode = usingMach ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS;
            const accelFactor = Common.getAccelerationFactor(mach, midStepAltitude, isaDev, midStepAltitude > tropoAltitude, accelFactorMode);
            const pathAngle = FlightModel.getConstantThrustPathAngleFromCoefficients(
                thrust,
                midStepWeight,
                liftCoefficient,
                dragCoefficient,
                accelFactor,
            );

            verticalSpeed = 101.268 * tas * Math.sin(pathAngle); // in feet per minute
            stepTime = stepSize / verticalSpeed; // in minutes
            distanceTraveled = (tas - headwindAtMidStepAlt) * stepTime;
            fuelBurned = (fuelFlow / 60) * stepTime;
            // const endStepWeight = zeroFuelWeight + (initialFuelWeight - fuelBurned);

            // Adjust variables for better accuracy next iteration
            previousMidStepWeight = midStepWeight;
            midStepWeight = zeroFuelWeight + (initialFuelWeight - (fuelBurned / 2));
            lift = midStepWeight * Math.cos(pathAngle);
            iterations++;
        } while (iterations < 5 && Math.abs(previousMidStepWeight - midStepWeight) < 100);

        let result: StepResults;
        result.pathAngle = pathAngle;
        result.verticalSpeed = verticalSpeed;
        result.timeElapsed = stepTime;
        result.distanceTraveled = distanceTraveled;
        result.fuelBurned = fuelBurned;
        return result;
    }

    static levelDistanceStep(
        altitude: number,
        stepSize: number,
        econCAS: number,
        econMach: number,
        zeroFuelWeight: number,
        initialFuelWeight: number,
        headwind: number,
        isaDev: number,
    ): StepResults {
        const theta = Common.getTheta(altitude, isaDev);
        const delta = Common.getDelta(theta);
        let mach = Common.CAStoMach(econCAS, delta);

        let tas;
        // If above crossover altitude, use econMach
        if (mach > econMach) {
            mach = econMach;
            tas = Common.machToTAS(mach, theta);
        } else {
            tas = Common.CAStoTAS(econCAS, theta, delta);
        }

        const initialWeight = zeroFuelWeight + initialFuelWeight;
        const thrust = FlightModel.getDrag(initialWeight, mach, delta, false, false, FlapConf.CLEAN);

        // Engine model calculations
        const theta2 = Common.getTheta2(theta, mach);
        const delta2 = Common.getDelta2(delta, mach);
        // Divide by 2 to get thrust per engine
        const correctedThrust = (thrust / delta2) / 2;
        // Since table 1506 describes corrected thrust as a fraction of max thrust, divide it
        const correctedN1 = EngineModel.reverseTableInterpolation(EngineModel.table1506, mach, (correctedThrust / EngineModel.maxThrust));
        const correctedFuelFlow = EngineModel.getCorrectedFuelFlow(correctedN1, mach, altitude) * 2;
        const fuelFlow = EngineModel.getUncorrectedFuelFlow(correctedFuelFlow, delta2, theta2); // in lbs/hour

        const stepTime = ((tas - headwind) / stepSize) / 60; // in minutes
        const fuelBurned = (fuelFlow / 60) * stepTime;

        let result: StepResults;
        result.pathAngle = 0;
        result.verticalSpeed = 0;
        result.timeElapsed = stepTime;
        result.distanceTraveled = stepSize;
        result.fuelBurned = fuelBurned;
        return result;
    }
}
