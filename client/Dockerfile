# stage: 1
FROM node:latest as builder

# Create app directory
WORKDIR /app

COPY . ./

RUN yarn install
RUN yarn build
CMD ["yarn","start"]
# stage: 2 — the production environment
FROM mhart/alpine-node
RUN yarn global add serve
WORKDIR /app
EXPOSE 8080 80
COPY --from=builder /app/build .
CMD ["serve", "-p", "80", "-s", "."]

