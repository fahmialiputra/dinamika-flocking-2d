let Engine = Matter.Engine,
    World = Matter.World,
    Body = Matter.Body,
    Bodies = Matter.Bodies;

let engine, world;
let boids = [], obs = [];
let num = 16,
    numO = 1;
let t = 0;

let cFrame, cCapt = false;
const capturer = new CCapture( {
    format: "webm",
    name: "Dinamika Flocking 2D",
    framerate: 60,
    quality: 100,
    verbose: false
} );

function setup() {
    cFrame = createCanvas(462, 400);
    engine = Engine.create();
    world = engine.world;
    world.gravity.y = 0;
    for(let i = 0; i < num; i++) { boids.push(new Boid(i)); }
    for(let i = 0; i < numO; i++) { obs.push(new Obstacle(width/2, height/2, 50)); }
}

function draw() {
    background(50);
    let last = 0;    
    for (i = 1; i < boids.length; i++) {
        if(boids[i].body.position.x < boids[last].body.position.x) last = i;
    }
    for(let o of obs) { o.show(); }
    for(let boid of boids) {
        let boidColor = color(255, 255, 255);
        if( boid.leader ) {
            boidColor = color(0, 255, 255);
            setGoal(); // set the Goal coordinate
        }
        boid.update(boids, obs);
        boid.show(boidColor);
    }
    Engine.update(engine, 0.1);
    if(frameCount === 1 && cCapt) { capturer.start(); }
    capturer.capture(cFrame.canvas);
    stopper();
    if(frameCount >= 60*15 && cCapt) { stopSimulation(); }
    t++;
}

function setGoal() {
    let r = 10;
    let x = mouseX || width/2;
    if( x > width) x = width;
    if( x < 0 ) x = 0;
    let y = mouseY || height/2;
    if( y > height ) y = height;
    if( y < 0 ) y = 0;
    // World.remove(world, obs[0].body);
    // obs.pop();
    // obs.push(new Obstacle(x, y, 10));
    for(boid of boids) {
        if(boid.leader) {
            let c = createVector(x, y);
            let d = boid.rij(boid.pos, c);
            boid.vd.mult(0);
            if(d.mag() <= r) {
                boid.vd.add(d.normalize());
                boid.vd.mult(1000); // the big number just to make sure the goal is aimed by
            } else {
                boid.vd.add(d.normalize());
            }
        }
    }
    push();
        strokeWeight(0.5);
        stroke(0, 255, 0, 250);
        noFill();
        translate(x, y);
        circle(0, 0, r*2);
    pop();
}

function stopSimulation() {
    noLoop();
    capturer.stop();
    capturer.save();
    print('Process Terminated');
}

function stopper() {
    /**
     * This function is used to make sure the simulation stopped
     * when all of the boids are
     * out of the area
     */
    let s = [0, 0, 0, 0];
    for(let boid of boids) {
        if(boid.pos.y < boids[s[0]].pos.y) s[0] = boid.index;
        if(boid.pos.y > boids[s[1]].pos.y) s[1] = boid.index;
        if(boid.pos.x < boids[s[2]].pos.x) s[2] = boid.index;
        if(boid.pos.x > boids[s[3]].pos.x) s[3] = boid.index;
    }
    if(boids[s[0]].pos.y > height + boids[s[0]].sensing) stopSimulation();
    if(boids[s[1]].pos.y < -boids[s[0]].sensing) stopSimulation();
    if(boids[s[2]].pos.x > width + boids[s[0]].sensing) stopSimulation();
    if(boids[s[3]].pos.x < -boids[s[0]].sensing) stopSimulation();
}