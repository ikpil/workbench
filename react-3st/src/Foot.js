import React from 'react';
import { Link } from 'react-router-dom'
import { Header, Grid, Container } from 'semantic-ui-react'

function Foot() {
  return (
    <Container>
      <Grid className='ui equal width center aligned grid container' id='colors'>
        <Grid.Row>
          <Grid.Column>
            <Link to="/">
              <Header icon>
                <i class="book icon"></i>
                
                <p>Home</p>
              </Header>
            </Link>
          </Grid.Column>
          <Grid.Column>
            <Link to="/pay">
              <Header icon>
                <i class="balance scale icon"></i>
                <p>Pay</p>
              </Header>
            </Link>
          </Grid.Column>
          <Grid.Column>
            <Link to="/order">
              <Header icon>
                <i class="birthday cake icon"></i>
                <p>Order</p>
              </Header>
            </Link>
          </Grid.Column>
          <Grid.Column>
            <Link to="/shop">
              <Header icon>
                <i class="address book icon"></i>
                <p>Shop</p>
              </Header>
            </Link>
          </Grid.Column>
          <Grid.Column>
            <Link to="/ohter">
              <Header icon>
                <i class="archive icon"></i>
                <p>Other</p>
              </Header>
            </Link>
          </Grid.Column>
        </Grid.Row>
      </Grid>
    </Container>
  );
}

export default Foot;
