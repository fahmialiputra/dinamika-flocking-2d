class Obstacle {

    color = color(255,0,0, 200);
    alphaLJ = 0.975;
    eObs = 0.01 * 2;

    constructor(x, y, radius) {
        this.body = Bodies.circle(x, y, radius, {
            friction: 0,
            frictionStatic: 0,
            isStatic: true
        });
        this.body.restitution = 0;
        this.body.slop = 0;
        this.body.mass = 1;
        this.pos = this.body.position;
        this.radius = this.body.circleRadius;
        World.add(world, this.body);
    }
  
    show() {
        noStroke();
        fill(this.color);
        circle(this.pos.x, this.pos.y, this.radius*2);
    }
}