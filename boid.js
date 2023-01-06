class Boid {

    maxSpeed = 1;
    v0 = 1;
    alpha = 0.5;
    betha = 1.2;
    k = 1;
    Dr = 0;
    Dt = 0;
    unitvector = createVector(1, 0);
    unitvector_perpendicular = createVector(0, 1);
    vd = createVector(1, 0);
    wg = 10;
    Er = createVector(random(0, 0.1), random(0, 0.1));
    Et = random(-PI/12, PI/12);
    leader = false;

    constructor(index) {
        this.index = index;
        if( this.index === 0 ) this.leader = true;
        this.body = Bodies.circle(width/4, random(10, height-10) + 10*index, 10);
        this.body.restitution = 0;
        this.body.slop = 0;
        this.body.mass = 1;
        this.pos = this.body.position;
        this.radius = this.body.circleRadius;
        this.sensing = this.radius*2;
        this.lij = this.radius*5;
        this.body.maxSpeed = this.maxSpeed;
        World.add(world, this.body);
    }

    drawLine(l) {
        push();
            strokeWeight(1);
            stroke(255);
            translate(this.pos.x, this.pos.y);
            line(0, 0, l.x, l.y);
        pop();
    }

    drawForceLine(F, linecolor) {
        let Fline = createVector(F.x, F.y);
        Fline.setMag(this.radius);
        push();
            strokeWeight(1);
            stroke(linecolor);
            translate(this.pos.x, this.pos.y);
            translate(this.radius*cos(Fline.heading()), this.radius*sin(Fline.heading()));
            line(0, 0, Fline.x, Fline.y);
        pop();
    }

    rij(i, j) {
        return p5.Vector.sub(createVector(j.x, j.y),
                             createVector(i.x, i.y));
    }

    equilibrium(boids) {
        let r0 = createVector();
        for (let other of boids) {
            r0.add(this.rij(this.pos, other.pos));
        }
        r0.div(boids.length);
        return r0;
    }

    Fi(boids) {
        let F = createVector();
        for (let other of boids) {
            if(other.leader) { this.k = 10; }
            else { this.k = 0.001; }
            if(other !== this) {
                let r = this.rij(this.pos, other.pos);
                let r1 = createVector(other.radius*cos(r.heading()), other.radius*sin(r.heading()));
                r.sub(r1);
                r.mult(-this.k*((1/r.mag()) - (1/this.lij)));
                F.add(r);
            }
        }
        return F;
    }

    Fg() {
        return p5.Vector.mult(this.vd, this.wg);
    }

    check(obs) {
        let checked = [];
        for (let o of obs) {
            let d = dist(this.pos.x, this.pos.y, o.pos.x, o.pos.y);
            if( d <= this.sensing + o.radius ) {
                checked.push(o);
            }
        }
        return checked;
    }

    Fobs(obs) {
        let F = createVector();
        let sigmaObs = this.sensing*sqrt(2);
        for (let o of obs) {
            let r = this.rij(this.pos, o.pos);
            let r1 = createVector(o.radius*cos(r.heading()), o.radius*sin(r.heading()));
            r.sub(r1);
            // let r2 = createVector(this.radius*cos(r.heading()), this.radius*sin(r.heading()));
            // r.sub(r2);
            r.mult(o.eObs*(
                pow(sigmaObs/r.mag(), 2*o.alphaLJ) -
                2 * pow(sigmaObs/r.mag(), o.alphaLJ)
            ));
            F.add(r);
        }
        return F;
    }

    Ftotal(boids, obs) {
        let F = createVector();
        if(!this.leader) {
            F.add(this.Fi(boids));
        } else {
            F.add(this.Fg());
        }
        F.add(this.Fobs(obs));
        this.drawForceLine(F, color(0, 255, 0));
        return F;
    }
    
    xi(Fi) {
        return this.v0 + this.alpha*p5.Vector.dot(
                                        p5.Vector.add(
                                            Fi,
                                            p5.Vector.mult(this.Er, this.Dr)
                                        ),
                                        this.unitvector
                                    );
    }

    thetai(Fi) {
        return this.betha*p5.Vector.dot(
                                p5.Vector.add(
                                    Fi,
                                    p5.Vector.mult(this.Er, this.Dr)
                                ),
                                this.unitvector_perpendicular
                            ) + this.Dt*this.Et;
    }
    
    update(boids, obs) {
        let F = this.Ftotal(boids, this.check(obs));
        Body.applyForce(this.body, this.pos, F);
        this.heading = createVector(this.body.velocity.x, this.body.velocity.y).heading();
    }

    show(color) {
        push();
            translate(this.pos.x, this.pos.y);
            rotate(this.heading);
            stroke(color);
            line(0, 0, this.radius, 0);
            noFill();
            circle(0, 0, this.radius*2);
        pop();
    }
}