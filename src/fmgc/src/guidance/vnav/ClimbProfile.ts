import { WayPoint } from '@fmgc/types/fstypes/FSTypes';

import { LateralMode, VerticalMode } from '@shared/autopilot';
import { ManagedFlightPlan } from '@fmgc/flightplanning/ManagedFlightPlan';
import { FlightPlanManager } from '@fmgc/flightplanning/FlightPlanManager';
import { SegmentType } from '@fmgc/flightplanning/FlightPlanSegment';
import { GuidanceComponent } from '../GuidanceComponent';
import { Leg, TFLeg } from '../Geometry';
import { GuidanceController } from '../GuidanceController';

// Local imports
import { Common, FlapConf } from './common';
import { EngineModel } from './EngineModel';
import { FlightModel } from './FlightModel';
import { Predictions } from './Predictions';

export class ClimbProfile implements GuidanceComponent {
    private guidanceController: GuidanceController;

    private fpm: FlightPlanManager;

    private activeWaypoint: WayPoint;

    private initialAltitude: number;

    private thrustReductionAltitude: number;

    private accelerationAltitude: number;

    private v2speed: number;

    private climbCAS: number;

    private climbMach: number;

    private zeroFuelWeight: number;

    private fuelWeight: number;

    private targetAltitude: number;

    private tropoAltitude: number;

    private isaDeviation: number;

    private fpChecksum: number;

    public distanceFromPposToSpdLim: number;

    public distanceFromPposToEarlyLeveloff: number;

    public distanceFromPposToStartOfClimb: number;

    public distanceFromPposToTopOfClimb: number;

    constructor(
        guidanceController: GuidanceController,
        fpm: FlightPlanManager,
        flightLevel: number,
        flightLevelTemp: number,
        tropoAltitude: number,
        zeroFuelWeight: number,
        fuelWeight: number,
        originWaypoint: WayPoint,
        thrustReductionAltitude: number,
        accelerationAltitude:number,
        v2speed: number,
        climbCAS: number,
        climbMach: number,
    ) {
        this.guidanceController = guidanceController;
        this.fpm = fpm;
        this.activeWaypoint = originWaypoint;
        // TODO: Field elevation -> initial altitude, especially if active runway hasn't been selected yet

        this.targetAltitude = flightLevel * 100;
        this.tropoAltitude = tropoAltitude;
        this.isaDeviation = Math.round(flightLevelTemp - Common.getIsaTemp(this.targetAltitude, this.targetAltitude > this.tropoAltitude));

        this.zeroFuelWeight = zeroFuelWeight;
        this.fuelWeight = fuelWeight;

        this.climbCAS = climbCAS;
        this.climbMach = climbMach;

        this.v2speed = v2speed;
        this.thrustReductionAltitude = thrustReductionAltitude;
        this.accelerationAltitude = accelerationAltitude;

        // The checksum to compare against the flight plan.
        this.fpChecksum = -1;

        // Results
        this.distanceFromPposToSpdLim = undefined;
        this.distanceFromPposToEarlyLeveloff = undefined;
        this.distanceFromPposToStartOfClimb = undefined;
        this.distanceFromPposToTopOfClimb = undefined;
    }

    init(): void {
        // console.log('[FMGC/Guidance] ClimbProfile initialized!');
    }

    update(_deltaTime: number): void {
        const geometry = this.guidanceController.guidanceManager.getMultipleLegGeometry();
        // const currentLeg = geometry.legs.get(1);

        // Check if conditions are met to start climb profile predictions

        // Update distance between PPOS and pseudo-waypoints (subtract delta distance from last frame)

        // Recompute if lateral/vertical revision or x number of seconds have passed since last recomputation
        // or if passing a constraint waypoint
        this.recompute(geometry.legs);
    }

    recompute(legs: Map<number, Leg>, climbN1: number):void {
        const belowAltitudeConstraints: Map<Leg, number>;
        const speedConstraints: Map<leg, number>;

        for (let leg of legs) {
            // iterate over legs to get alt & speed constraints
            // Constraint applies to the terminator of each leg (dest waypoint)
            // Process conflicting constraints as well
        }

        // TODO: If below field elevation + accel altitude, perform necessary calculations to find distance from takeoff to accel altitude

        const firstStepSize = (Math.ceil((this.initialAltitude + 100) / 1000) * 1000) - this.initialAltitude;

        let finalStepAltitude;
        let finalStepSize;
        if (this.targetAltitude % 1000 === 0) {
            finalStepAltitude = this.targetAltitude - 1000;
            finalStepSize = 1000;
        } else {
            finalStepAltitude = (Math.floor((this.targetAltitude - 100) / 1000) * 1000);
            finalStepSize = this.targetAltitude - finalStepAltitude;
        }

        // First step
        const firstStepResult = Predictions.altitudeStep(
            this.initialAltitude,
            firstStepSize,
            this.climbCAS,
            this.climbMach,
            climbN1,
            this.zeroFuelWeight,
            this.fuelWeight,
            0,
            this.isaDeviation,
            this.tropoAltitude,
        );

        let predDistanceTraveled = firstStepResult.distanceTraveled;
        let predTimeElapsed = firstStepResult.timeElapsed;
        let predFuelWeight = this.fuelWeight - firstStepResult.fuelBurned;

        // Loop (ignoring constraints for now)
        // TODO: stop at each waypoint as well as SPD LIM, and record time elapsed and fuel weight
        for (let alt = this.initialAltitude + firstStepSize; alt < finalStepAltitude; alt += 1000) {
            const stepResult = Predictions.altitudeStep(
                alt,
                1000,
                this.climbCAS,
                this.climbMach,
                climbN1,
                this.zeroFuelWeight,
                predFuelWeight,
                0,
                this.isaDeviation,
                this.tropoAltitude,
            );
            predDistanceTraveled += stepResult.distanceTraveled;
            predTimeElapsed += stepResult.timeElapsed;
            predFuelWeight -= stepResult.fuelBurned;
        }

        const finalStepResult = Predictions.altitudeStep(
            finalStepAltitude,
            finalStepSize,
            this.climbCAS,
            this.climbMach,
            climbN1,
            this.zeroFuelWeight,
            predFuelWeight,
            0,
            this.isaDeviation,
            this.tropoAltitude,
        );
        predDistanceTraveled += finalStepResult.distanceTraveled;
        predTimeElapsed += finalStepResult.timeElapsed;
        predFuelWeight -= finalStepResult.fuelBurned;

        this.distanceFromPposToTopOfClimb = predDistanceTraveled;
    }

    /**
     * The active flight plan.
     * @type {ManagedFlightPlan}
     */
    get flightplan(): ManagedFlightPlan {
        return this.fpm.getFlightPlan(0);
    }

    get currentWaypoints(): WayPoint[] {
        return this.flightplan.waypoints.slice(this.flightplan.activeWaypointIndex);
    }

    get allWaypoints(): WayPoint[] {
        return this.flightplan.waypoints;
    }
}
