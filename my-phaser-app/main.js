
import Phaser from 'phaser';
import * as Box2D from 'phaser-box2d/dist/PhaserBox2D.js';


const config = {
  type: Phaser.AUTO,
  width: 800,
  height: 600,
  backgroundColor: '#222',
  physics: {
    default: 'arcade', // Use arcade for a simple demo
    arcade: {
      gravity: { y: 300 },
      debug: true
    }
  },
  scene: {
    preload,
    create,
    update
  }
};


function preload() {
  this.load.image('ball', 'https://labs.phaser.io/assets/sprites/shinyball.png');
}

function create() {
  // Add a ball sprite and enable physics
  this.ball = this.physics.add.image(400, 100, 'ball');
  this.ball.setBounce(0.8);
  this.ball.setCollideWorldBounds(true);
}

function update() {}

new Phaser.Game(config);
